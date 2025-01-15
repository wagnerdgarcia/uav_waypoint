cmake_minimum_required(VERSION 3.8)                         # Define a versão mínima necessária do CMake para compilar o projeto
project(uav_waypoint)                                       # Define o nome do projeto, neste caso 'uav_waypoint'

# Verifica se o padrão C++ não foi definido e, se não, define o padrão C++17 como obrigatório
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)                                # Define a versão do C++ para 17
  set(CMAKE_CXX_STANDARD_REQUIRED ON)                       # Garante que o C++17 será utilizado
endif()

# Adiciona opções de compilação específicas para compressores GCC ou Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)             # Ativa avisos extra e pedantes para garantir um código limpo
endif()

# Encontra e inclui os pacotes necessários para compilar o código
find_package(ament_cmake REQUIRED)                          # Pacote de construção do ROS2
find_package(rclcpp REQUIRED)                               # Biblioteca básica para programação de nós ROS2
find_package(rclcpp_lifecycle REQUIRED)                     # Pacote para gerenciar o ciclo de vida dos nós
find_package(rclcpp_components REQUIRED)                    # Pacote para trabalhar com componentes de nós
find_package(geometry_msgs REQUIRED)                        # Mensagens de geometria para representar pontos, vetores, etc.
find_package(nav_msgs REQUIRED)                             # Pacote de mensagens para navegação de robôs
find_package(std_srvs REQUIRED)                             # Pacote para definir e chamar serviços padrão no ROS2
find_package(tf2 REQUIRED)                                  # Pacote para transformações de coordenadas no ROS2
find_package(tf2_ros REQUIRED)                              # Pacote ROS2 que usa TF2 para transformações
find_package(tf2_geometry_msgs REQUIRED)                    # Pacote para converter entre tipos de geometria e transformações

# Define uma lista de dependências que serão usadas em todo o projeto
set(dependencies
  rclcpp                                                   # Dependência principal para criar nós ROS2
  rclcpp_lifecycle                                         # Dependência para ciclo de vida do nó
  rclcpp_components                                        # Dependência para componentes de nós ROS2
  nav_msgs                                                 # Dependência para mensagens de navegação
  std_srvs                                                 # Dependência para serviços padrão
  tf2                                                      # Dependência para transformações espaciais
  tf2_ros                                                  # Dependência para ROS2 com transformações
  tf2_geometry_msgs                                        # Dependência para converter transformações com mensagens de geometria
)

# Adiciona o diretório 'include' para que os arquivos de cabeçalho sejam encontrados pelo compilador
include_directories(
  include
)

# Define o nome do executável que será gerado a partir do código principal
set(executable_name waypoint_loader)

# Cria o executável 'waypoint_loader' a partir do código-fonte 'waypoint_loader_main.cpp'
add_executable(${executable_name}
  src/waypoint_loader_main.cpp
)

# Define o nome da biblioteca compartilhada que será gerada
set(library_name waypoint_loader_node)

# Cria a biblioteca compartilhada 'waypoint_loader_node' a partir do código-fonte 'waypoint_loader.cpp'
add_library(${library_name} SHARED
  src/waypoint_loader.cpp
)

# Vincula as dependências necessárias tanto ao executável quanto à biblioteca
ament_target_dependencies(${executable_name}
  ${dependencies}
)

# Vincula a biblioteca ao executável para que ele possa usar a funcionalidade da biblioteca
target_link_libraries(${executable_name}
  ${library_name}
)

# Vincula as dependências ao projeto da biblioteca
ament_target_dependencies(${library_name}
  ${dependencies}
)

# Especifica que a biblioteca depende do diretório 'src/include' para incluir arquivos de cabeçalho
target_include_directories(${library_name} PRIVATE src/include)

# Registra o nó 'WaypointLoader' como um componente do ROS2, que pode ser carregado dinamicamente
rclcpp_components_register_nodes(${library_name} "uav_waypoint::WaypointLoader")

# Define onde os arquivos gerados serão instalados no sistema
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib                                   # Arquivos de biblioteca vão para 'lib'
  LIBRARY DESTINATION lib                                   # Bibliotecas vão para 'lib'
  RUNTIME DESTINATION bin                                   # Executáveis vão para 'bin'
)

# Instala o executável no diretório do pacote
install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}                   # Executáveis vão para 'lib/${PROJECT_NAME}'
)

# Instala os arquivos de cabeçalho na pasta 'include'
install(DIRECTORY include/
  DESTINATION include/                                      # Arquivos de cabeçalho vão para 'include/'
)

# Instala as pastas 'launch' e 'params' no diretório compartilhado do projeto
install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}                          # 'launch' e 'params' vão para 'share/${PROJECT_NAME}'
)

# Se a compilação de testes estiver habilitada, configura ferramentas de linting e verificação de estilo de código
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)                     # Encontra pacotes para automatizar verificações de estilo de código
  set(ament_cmake_lint_cmake_FOUND TRUE)                     # Garante que o linting de CMake seja encontrado
  set(ament_cmake_cpplint_FOUND TRUE)                        # Garante que o linting de C++ seja encontrado
  ament_lint_auto_find_test_dependencies()                   # Encontra dependências para linting de código
endif()

# Exporta diretórios de inclusão e dependências do projeto para outros pacotes
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

# Finaliza a configuração do pacote ROS2
ament_package()
