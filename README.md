# uav_waypoint

`uav_waypoint` é um pacote ROS2 que implementa um nó de carregamento de waypoints para o gerenciamento de waypoints de UAV (Veículo Aéreo Não Tripulado). Este pacote permite carregar, gerenciar e enviar waypoints para o UAV de forma eficiente.

## Funcionalidade

O nó `waypoint_loader` é responsável por carregar uma sequência de waypoints, controlar o estado de execução e enviar as mensagens de destino para o UAV. Ele é configurável e pode ser ajustado para diferentes cenários de missão.

## Requisitos

Este pacote requer o ROS 2 (versão humble) e as seguintes dependências:

- `rclcpp` - Biblioteca principal para criar nós ROS2.
- `rclcpp_lifecycle` - Para gerenciamento do ciclo de vida dos nós.
- `rclcpp_components` - Para trabalhar com componentes ROS2.
- `geometry_msgs` - Para mensagens de geometria (pose, ponto, etc).
- `nav_msgs` - Mensagens para navegação no ROS2.
- `std_srvs` - Para serviços padrão no ROS2.
- `tf2`, `tf2_ros`, `tf2_geometry_msgs` - Para lidar com transformações de coordenadas.

## Instalação

Para usar o pacote `uav_waypoint`, siga as instruções abaixo:

### 1. Clone o repositório

Clone o repositório para a pasta `src` do seu workspace ROS2.

```bash
cd ~/ros2_ws/src
git clone https://github.com/wagnerdgarcia/uav_waypoint.git
```

### 2. Instale as dependências

Instale as dependências necessárias com o seguinte comando:

```bash
cd ~/laser_uav_system_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Construa o workspace

Construa o workspace usando o `colcon`.

```bash
colcon build --symlink-install
```

### 4. Fonte o ambiente

Após a construção, fonte o ambiente:

```bash
source ~/laser_uav_system_ws/install/setup.bash
```

## Uso

### 1. Lançamento do nó

O pacote `uav_waypoint` contém um nó chamado `waypoint_loader`, que pode ser executado com o seguinte comando:

```bash
ros2 launch uav_waypoint waypoint_loader_launch.py
```

### 2. Configuração

O pacote usa parâmetros definidos em um arquivo YAML. Exemplo de configuração:

```yaml
/uav1/waypoint_loader:
  ros__parameters:
    rate:
      state_machine: 0.5  # [Hz] Taxa de atualização da máquina de estados
      have_goal: 0.5      # [Hz] Taxa de verificação de objetivo
    trajectory:
      loop: true           # Defina como true para um loop infinito dos waypoints
      waypoints: [
        0.0,  0.0, 2.0, 0.0, 
        1.0, 0.0, 2.0, 0.0, 
        1.0, 1.0, 2.0, 0.0, 
        0.0, 1.0, 2.0, 0.0, 
      ]  # Waypoints definidos como [x, y, z, yaw]
```

A configuração do arquivo YAML pode ser carregada no seu launch file.

### 3. Serviços

O nó oferece o serviço `start_state_machine` para iniciar a máquina de estados, o que pode ser feito com o seguinte comando:

```bash
ros2 service call /uav1/waypoint_loader/start_state_machine std_srvs/srv/Trigger
```

## Estrutura do Pacote

A estrutura do pacote é a seguinte:

```
uav_waypoint/
├── CMakeLists.txt        # Arquivo de configuração de construção do CMake
├── package.xml           # Arquivo de configuração do pacote ROS2
├── launch/               # Arquivos de lançamento (launch)
├── params/               # Arquivos de parâmetros (YAML)
├── src/                  # Código-fonte
│   ├── waypoint_loader.cpp  # Implementação do nó de waypoint_loader
│   └── waypoint_loader_main.cpp  # Ponto de entrada principal
└── include/              # Arquivos de cabeçalho
```

## Contribuição

Sinta-se à vontade para contribuir com este projeto. Para isso:

1. Faça um fork deste repositório.
2. Crie uma branch para sua feature (`git checkout -b feature/nome-da-feature`).
3. Faça suas modificações.
4. Envie um pull request.

## Licença

Este pacote é licenciado sob a licença MIT.

## Contato

Para mais informações ou dúvidas, entre em contato com o mantenedor:

- **Wagner Garcia** - wagnergarcia@eng.ci.ufpb.br
