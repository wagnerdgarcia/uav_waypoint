#ifndef UAV_WAYPOINT__WAYPOINT_LOADER_HPP_
#define UAV_WAYPOINT__WAYPOINT_LOADER_HPP_

/* includes //{ */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <memory>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
/*//}*/

/* define //{*/
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
/*//}*/

namespace uav_waypoint
{
    /**
     * @class WaypointLoader
     * @brief Classe principal para carregar e gerenciar waypoints de um UAV.
     */
    class WaypointLoader : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        /**
         * @brief Construtor da classe WaypointLoader.
         * @param options Opções do nó ROS 2.
         */
        /* WaypointLoader() //{ */
        explicit WaypointLoader(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        /*//}*/

        /**
         * @brief Destrutor da classe WaypointLoader.
         */
        /* ~WaypointLoader() //{ */
        ~WaypointLoader() override;
        /*//}*/

    private:
        /* CONFIG //{ */

        /**
         * @brief Configura o nó no estado "configure".
         * @param state Estado atual do ciclo de vida.
         * @return Resultado da configuração.
         */
        /* on_configure() //{ */
        CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
        /*//}*/

        /**
         * @brief Ativa o nó no estado "activate".
         * @param state Estado atual do ciclo de vida.
         * @return Resultado da ativação.
         */
        /* on_activate() //{ */
        CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
        /*//}*/

        /**
         * @brief Desativa o nó no estado "deactivate".
         * @param state Estado atual do ciclo de vida.
         * @return Resultado da desativação.
         */
        /* on_deactivate() //{ */
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
        /*//}*/

        /**
         * @brief Limpa os recursos no estado "cleanup".
         * @param state Estado atual do ciclo de vida.
         * @return Resultado da limpeza.
         */
        /* on_cleanup() //{ */
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
        /*//}*/

        /**
         * @brief Realiza ações no estado "shutdown".
         * @param state Estado atual do ciclo de vida.
         * @return Resultado do desligamento.
         */
        /* on_shutdown() //{ */
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
        /*//}*/

        /**
         * @brief Obtém parâmetros configurados no arquivo de configuração.
         */
        /* getParameters() //{ */
        void getParameters();
        /*//}*/

        /**
         * @brief Configura publishers e subscribers para comunicação ROS.
         */
        /* configPubSub() //{ */
        void configPubSub();
        /*//}*/

        /**
         * @brief Configura os timers para execução periódica de tarefas.
         */
        /* configTimers() //{ */
        void configTimers();
        /*//}*/

        /**
         * @brief Configura os serviços oferecidos pelo nó.
         */
        /* configServices() //{ */
        void configServices();
        /*//}*/
        /*//}*/

        /* SUBSCRIBERS //{ */
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_have_goal_;

        /**
         * @brief Callback chamado quando uma mensagem indicando a existência de um objetivo é recebida.
         * @param msg Mensagem recebida.
         */
        /* subHaveGoal() //{ */
        void subHaveGoal(const std_msgs::msg::Bool::SharedPtr msg);
        /*//}*/

        /*//}*/

        /* PUBLISHERS //{ */
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose>::SharedPtr pub_goto_;
        /*//}*/

        /* VARIAVEIS GLOBAIS //{ */
        std::vector<double> _waypoints_values_;                     ///< Valores dos waypoints.
        std::vector<geometry_msgs::msg::Pose> load_waypoints;       ///< Lista de waypoints carregados.
        std::vector<geometry_msgs::msg::Pose> load_waypoints_save_; ///< Backup dos waypoints carregados.

        bool _trajectory_loop_{false}; ///< Indica se o trajeto deve ser repetido.
        double _rate_have_goal_;       ///< Taxa de atualização para verificar a existência de um objetivo.
        double _rate_state_machine_;   ///< Taxa de execução da máquina de estados.

        std::mutex mtx_;            ///< Mutex para sincronização de acesso.
        bool have_goal_{false};     ///< Indica se há um objetivo ativo.
        bool is_active_{false};     ///< Indica se o nó está ativo.
        bool wait_for_start_{true}; ///< Indica se está aguardando início.

        // Estados da máquina de estados
        enum States
        {
            GO_POINT, ///< Estado para ir a um ponto.
            WAIT,     ///< Estado de espera.
            FINISH    ///< Estado final.
        };

        unsigned int current_state_;                      ///< Estado atual da máquina.
        std::vector<States> machine_states_ = {GO_POINT}; ///< Sequência de estados da máquina.

        /*//}*/

        /* TIMERS //{ */

        /**
         * @brief Controlador principal da máquina de estados.
         */
        /* tmrStateMachineController() //{ */
        void tmrStateMachineController();
        /*//}*/

        rclcpp::TimerBase::SharedPtr tmr_state_machine_;
        /*//}*/

        /* SERVICES //{ */
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_state_machine_;

        /**
         * @brief Callback para iniciar a máquina de estados via serviço.
         * @param request Requisição do serviço.
         * @param response Resposta do serviço.
         */
        /* srvStartStateMachine() //{ */
        void srvStartStateMachine(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        /*//}*/

        /*//}*/

        /* FUNCIONS //{ */
        /**
         * @brief Executa o estado de ir ao ponto atual.
         */
        /* goPoint() //{ */
        void goPoint();
        /*//}*/
        /*//}*/
    };
} // namespace uav_waypoint

#endif // UAV_WAYPOINT__WAYPOINT_LOADER_HPP_
