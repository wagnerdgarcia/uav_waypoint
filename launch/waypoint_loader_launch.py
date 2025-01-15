from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import RegisterEventHandler, EmitEvent
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.events import matches_action
from launch.event_handlers.on_process_start import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg

def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    # Argumento para definir o namespace do drone
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='uav1',  # Valor padrão: 'uav1'
            description='Top-level namespace for the UAV.'
        )
    )

    # Argumento para o arquivo de parâmetros do waypoint_loader
    declared_arguments.append(
        DeclareLaunchArgument(
            'waypoint_loader_file',
            default_value=PathJoinSubstitution([FindPackageShare('uav_waypoint'),
                                                'params', 'waypoint_loader.yaml']),
            description='Full path to the file with the waypoint loader parameters.'
        )
    )

    # Inicialização dos argumentos
    namespace = LaunchConfiguration('namespace')
    waypoint_loader_file = LaunchConfiguration('waypoint_loader_file')

    # Definição do LifecycleNode
    waypoint_lifecycle_node = LifecycleNode(
        package='uav_waypoint',  # Nome do pacote
        executable='waypoint_loader',  # Nome do executável
        name='waypoint_loader',  # Nome do nó
        namespace=namespace,  # Namespace do nó
        output='screen',  # Saída do log no terminal
        parameters=[waypoint_loader_file],  # Carrega os parâmetros do arquivo YAML
        remappings=[
            # Remapeamentos de tópicos
            ('have_goal', '/uav1/have_goal'),  # Remapeia tópico de entrada
            ('goto', '/uav1/goto'),  # Remapeia tópico de saída

            # Remapeamentos de serviços
            ('start_state_machine', '/uav1/waypoint_loader/start_state_machine'),  # Remapeia o serviço de carregamento de waypoints
        ]
    )

    # Lista de manipuladores de eventos
    event_handlers = []

    # Manipulador para realizar a transição para o estado 'configure' assim que o nó for iniciado
    event_handlers.append(
        RegisterEventHandler(
            OnProcessStart(
                target_action=waypoint_lifecycle_node,
                on_start=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(waypoint_lifecycle_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
                ],
            )
        ),
    )

    # Manipulador para realizar a transição para o estado 'activate' após o estado 'configuring'
    event_handlers.append(
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=waypoint_lifecycle_node,
                start_state='configuring',
                goal_state='inactive',
                entities=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(waypoint_lifecycle_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        ),
    )

    # Criação do LaunchDescription
    ld = LaunchDescription()

    # Declaração dos argumentos
    for argument in declared_arguments:
        ld.add_action(argument)

    # Adiciona o nó do waypoint_loader
    ld.add_action(waypoint_lifecycle_node)

    # Adiciona os manipuladores de eventos
    for event_handler in event_handlers:
        ld.add_action(event_handler)

    return ld
