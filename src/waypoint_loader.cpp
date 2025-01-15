#include <uav_waypoint/waypoint_loader.hpp>

namespace uav_waypoint
{
    /**
     * @brief Construtor da classe WaypointLoader.
     * Configura o nó, inicializa parâmetros e registra mensagens no log.
     * @param options Opções do nó ROS.
     */
    /* WaypointLoader() //{ */
    WaypointLoader::WaypointLoader(const rclcpp::NodeOptions &options)
        : rclcpp_lifecycle::LifecycleNode("waypoint_loader", "", options)
    {
        RCLCPP_INFO(get_logger(), "Creating");

        // Declaração dos parâmetros para serem carregados pelo nó
        declare_parameter("trajectory.waypoints", rclcpp::ParameterValue(std::vector<double>()));
        declare_parameter("trajectory.loop", rclcpp::ParameterValue(false));
        declare_parameter("rate.state_machine", rclcpp::ParameterValue(1.0));
        declare_parameter("rate.have_goal", rclcpp::ParameterValue(1.0));

        RCLCPP_INFO(get_logger(), "WaypointLoader node initialized.");
    }
    /*//}*/

    /**
     * @brief Destrutor da classe WaypointLoader.
     */
    /* ~WaypointLoader() //{ */
    WaypointLoader::~WaypointLoader() {}
    /*//}*/

    /**
     * @brief Configura o nó no estado "configure".
     * Inicializa parâmetros, publishers, subscribers, timers e serviços.
     * @return CallbackReturn::SUCCESS se configurado com sucesso.
     */
    /* on_configure() //{ */
    CallbackReturn WaypointLoader::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Configuring WaypointLoader...");

        getParameters();  // Carrega os parâmetros do nó.
        configPubSub();   // Configura os publishers e subscribers.
        configTimers();   // Configura os timers.
        configServices(); // Configura os serviços.

        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /**
     * @brief Ativa o nó no estado "activate".
     * Habilita o publisher e marca o nó como ativo.
     * @return CallbackReturn::SUCCESS se ativado com sucesso.
     */
    /* on_activate() //{ */
    CallbackReturn WaypointLoader::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Activating WaypointLoader...");

        pub_goto_->on_activate();

        {
            std::lock_guard<std::mutex> lock(mtx_);
            is_active_ = true;
        }

        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /**
     * @brief Desativa o nó no estado "deactivate".
     * Desabilita o publisher e marca o nó como inativo.
     * @return CallbackReturn::SUCCESS se desativado com sucesso.
     */
    /* on_deactivate() //{ */
    CallbackReturn WaypointLoader::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Deactivating WaypointLoader...");

        pub_goto_->on_deactivate();

        {
            std::lock_guard<std::mutex> lock(mtx_);
            is_active_ = false;
        }

        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /**
     * @brief Realiza limpeza de recursos no estado "cleanup".
     * Libera memória e recursos associados aos publishers e subscribers.
     * @return CallbackReturn::SUCCESS se limpo com sucesso.
     */
    /* on_cleanup() //{ */
    CallbackReturn WaypointLoader::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up WaypointLoader...");

        pub_goto_.reset();
        sub_have_goal_.reset();

        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /**
     * @brief Realiza ações de desligamento no estado "shutdown".
     * @return CallbackReturn::SUCCESS se desligado com sucesso.
     */
    /* on_shutdown() //{ */
    CallbackReturn WaypointLoader::on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Shutting down WaypointLoader...");
        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /**
     * @brief Carrega parâmetros configurados do arquivo de configuração do ROS 2.
     * Configura os waypoints, taxa de execução e outras opções.
     */
    /* getParameters() //{ */
    void WaypointLoader::getParameters()
    {
        get_parameter("rate.state_machine", _rate_state_machine_);
        get_parameter("rate.have_goal", _rate_have_goal_);
        get_parameter("trajectory.loop", _trajectory_loop_);
        get_parameter("trajectory.waypoints", _waypoints_values_);

        if (_waypoints_values_.size() % 4 != 0)
        {
            RCLCPP_ERROR(get_logger(), "Invalid number of waypoints. Must be multiple of 4.");
            return;
        }

        // Converte a lista de valores em poses 3D
        for (size_t i = 0; i < _waypoints_values_.size(); i += 4)
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = _waypoints_values_[i];
            pose.position.y = _waypoints_values_[i + 1];
            pose.position.z = _waypoints_values_[i + 2];

            double yaw = _waypoints_values_[i + 3];
            tf2::Quaternion quaternion;
            quaternion.setRPY(0.0, 0.0, yaw);
            pose.orientation.x = quaternion.x();
            pose.orientation.y = quaternion.y();
            pose.orientation.z = quaternion.z();
            pose.orientation.w = quaternion.w();

            load_waypoints.push_back(pose);
        }

        // Backup dos waypoints para reinicializações futuras
        load_waypoints_save_ = load_waypoints;

        RCLCPP_INFO(get_logger(), "Parameters loaded.");
    }
    /*//}*/

    /**
     * @brief Configura os publishers e subscribers para comunicação.
     */
    /* configPubSub() //{ */
    void WaypointLoader::configPubSub()
    {
        RCLCPP_INFO(get_logger(), "Initializing publishers and subscribers...");

        // Publisher para enviar comandos de waypoint
        pub_goto_ = create_publisher<geometry_msgs::msg::Pose>("goto", 10);

        // Subscriber para monitorar se há um objetivo ativo
        sub_have_goal_ = create_subscription<std_msgs::msg::Bool>(
            "have_goal", 10, std::bind(&WaypointLoader::subHaveGoal, this, std::placeholders::_1));
    }
    /*//}*/

    /**
     * @brief Configura os timers para execução periódica da lógica do nó.
     */
    /* configTimers() //{ */
    void WaypointLoader::configTimers()
    {
        RCLCPP_INFO(get_logger(), "Initializing timers...");

        tmr_state_machine_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / _rate_state_machine_),
            std::bind(&WaypointLoader::tmrStateMachineController, this));
    }
    /*//}*/

    /**
     * @brief Configura os serviços oferecidos pelo nó.
     *
     * Essa função inicializa os serviços que o nó disponibiliza. No caso, o serviço configurado é:
     * - `start_state_machine`: Permite que um cliente inicie a máquina de estados.
     *
     * O serviço utiliza a função de callback `srvStartStateMachine` para tratar as requisições recebidas.
     */
    /* configServices() //{ */
    void WaypointLoader::configServices()
    {
        RCLCPP_INFO(get_logger(), "Initializing services..."); // Loga mensagem indicando a inicialização dos serviços.

        // Cria o serviço "start_state_machine" com tipo std_srvs::srv::Trigger
        // Liga o serviço à função de callback srvStartStateMachine.
        srv_start_state_machine_ = create_service<std_srvs::srv::Trigger>(
            "start_state_machine",
            std::bind(&WaypointLoader::srvStartStateMachine, this, std::placeholders::_1, std::placeholders::_2));
    }

    /*//}*/

    /**
     * @brief Callback para o subscriber do tópico "have_goal".
     * @param msg Mensagem do tipo std_msgs::msg::Bool indicando se há um objetivo disponível.
     *
     * Essa função é chamada toda vez que uma mensagem é publicada no tópico "have_goal".
     * Atualiza a variável `have_goal_` com o valor recebido e garante a sincronização com mutex.
     */
    /* subHaveGoal() //{ */
    void WaypointLoader::subHaveGoal(const std_msgs::msg::Bool::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx_); // Protege o acesso a `have_goal_` durante a escrita.
        have_goal_ = msg->data;                 // Atualiza o estado de `have_goal_` com a mensagem recebida.
    }

    /*//}*/

    /**
     * @brief Serviço para iniciar a máquina de estados.
     * @param request Requisição do tipo std_srvs::srv::Trigger::Request.
     * @param response Resposta do tipo std_srvs::srv::Trigger::Response indicando o status do serviço.
     *
     * Verifica se a máquina de estados está ativa antes de permitir sua execução.
     * Se ativa, altera a variável `wait_for_start_` para permitir o início do processamento.
     */
    /* srvStartStateMachine() //{ */
    void WaypointLoader::srvStartStateMachine(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(mtx_); // Protege o acesso a `is_active_` durante a verificação.

        if (!wait_for_start_)
        {
            response->success = false; // Indica falha no início da máquina de estados.
            response->message = "State machine ready.";
            return;
        }

        if (!is_active_)
        {
            response->success = false; // Indica falha no início da máquina de estados.
            response->message = "State machine not active.";
            return;
        }
        else
        {
            response->success = true; // Início bem-sucedido da máquina de estados.
            response->message = "State machine started.";
            wait_for_start_ = false; // Atualiza a flag para iniciar o processamento.
        }
    }

    /*//}*/

    /**
     * @brief Controlador principal da máquina de estados.
     *
     * Essa função é chamada periodicamente pelo timer `tmr_state_machine_`.
     * Controla o fluxo entre os estados definidos em `machine_states_`.
     * Estados possíveis:
     * - GO_POINT: Executa o próximo waypoint.
     * - WAIT: Aguarda até que `have_goal_` seja Falso.
     * - FINISH: Finaliza a sequência e encerra o nó.
     */
    /* tmrStateMachineController() //{ */
    void WaypointLoader::tmrStateMachineController()
    {
        std::lock_guard<std::mutex> lock(mtx_); // Protege o acesso às variáveis da máquina de estados.
        if (wait_for_start_ || !is_active_)
        {
            return; // Se a máquina de estados não está pronta ou ativa, retorna sem executar nada.
        }

        if (!have_goal_ && machine_states_[current_state_] == WAIT) // Verifica se o estado atual é WAIT e se não há objetivo.
        {
            current_state_++; // Avança para o próximo estado.
        }

        switch (machine_states_[current_state_]) // Executa o estado atual da máquina.
        {
        case GO_POINT: // Estado para ir a um ponto.
            goPoint(); // Chama a função para ir ao próximo waypoint.
            break;
        case WAIT: // Estado de espera.
            break;
        case FINISH: //  Estado final.
            RCLCPP_INFO(get_logger(), "Waypoint sequence finished.");
            rclcpp::shutdown(); // Finaliza a execução do nó.
        }
    }
    /*//}*/

    /**
     * @brief Executa o próximo waypoint na sequência.
     *
     * Verifica se há waypoints disponíveis, publica o próximo waypoint no tópico "goto",
     * e gerencia o estado da máquina conforme necessário.
     */
    /* goPoint() //{ */
    void WaypointLoader::goPoint()
    {
        if (load_waypoints.empty())
        {
            RCLCPP_INFO(get_logger(), "No waypoints to go."); // Loga mensagem caso não haja waypoints.
            return;
        }

        geometry_msgs::msg::Pose pose = load_waypoints.front(); // Obtém o próximo waypoint.
        load_waypoints.erase(load_waypoints.begin());           // Remove o waypoint da lista.
        pub_goto_->publish(pose);                               // Publica o waypoint no tópico "goto".

        RCLCPP_INFO(get_logger(), "Going to waypoint: x=%f, y=%f, z=%f", pose.position.x, pose.position.y, pose.position.z);

        if (load_waypoints.empty())
        {
            if (_trajectory_loop_)
            {
                load_waypoints = load_waypoints_save_;  // Recarrega os waypoints salvos para repetir o loop.
                machine_states_.push_back(GO_POINT);    // Adiciona o estado GO_POINT para repetir o trajeto.
                machine_states_[current_state_] = WAIT; // Adiciona o estado WAIT para aguardar.
            }
            else
            {
                machine_states_.push_back(FINISH); // Adiciona o estado FINISH se não há mais waypoints.
            }
        }
        else
        {
            machine_states_.push_back(GO_POINT);    // Adiciona o estado GO_POINT para o próximo waypoint.
            machine_states_[current_state_] = WAIT; // Adiciona o estado WAIT para aguardar.
        }
    }

    /*//}*/
} // namespace uav_waypoint
