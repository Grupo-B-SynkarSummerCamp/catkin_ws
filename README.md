# Syncar Robot Simulation Workspace

Este repositório contém o workspace ROS (catkin) para a simulação do Robô Syncar, desenvolvido para o Summer Camp “Robótica Zero to Hero” de 2025.

---

## Objetivos

- Fornecer o ambiente completo de simulação do robô Syncar no Gazebo.
- Permitir controle remoto via teleop.
- Exibir vídeo da câmera Intel RealSense em tempo real.

---

## Dependências

Antes de usar este workspace, certifique-se de ter instalado:

1. **ROS Noetic**
   Siga o guia oficial:
   ```bash
   # configuração das chaves e repositórios
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
   sudo apt update
   sudo apt install ros-noetic-desktop-full
   ```
3. **Pacote de teleop**
   Para controle via teclado:
   ```bash
   sudo apt install ros-noetic-teleop-twist-keyboard
   ```
4. **Pacote RealSense**
   Para visualização da câmera Intel RealSense:
   ```bash
   sudo apt install ros-noetic-realsense2-*
   ```
5. **Baixe o teleop joy**
6. **Baixe o py evdev**
   ```
   sudo apt install python3-evdev
   ```

---

## Clonando o Repositório

Use o comando abaixo para clonar o workspace com todos os submódulos (recursivo):

```bash
git clone https://github.com/Grupo-B-SynkarSummerCamp/catkin_ws.git --recursive
```

---

## Compilando o Workspace

Use os comandos abaixo para compilar o repositorio

```bash
cd catkin_ws
catkin_make
```

Depois de compilar, não esqueça de executar o “source” do setup:

```bash
source devel/setup.bash
```

---

## Executando a Simulação

1. Inicie o ROS Master:
   ```bash
   roscore
   ```
2. Dentro do workspace execute o source, se não ja tiver feito:
   ```bash
   source devel/setup.bash
   ```
3. Em outra aba do terminal, lance a simulação no Gazebo:
   ```bash
   roslaunch synkar_robot_description gazebo.launch
   ```
4. Para teleop:
   ```bash
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py
   ```
5. Para visualizar o stream da RealSense use o RVIZ em outro terminal:
    ```bash
   rviz
   ```


---

## Executando no Hardware Real

Para usar o repositório com o robô Syncar de verdade, siga os passos abaixo:


1. No robô, inicie o ROS Master:
   ```bash
   roscore
   ```
2. Configure sua máquina para comunicação com o robô (em rede):
   ```bash
   rosrun rosserial_python serial_node.py _port:=tcp
   ```
3. No computador host, dentro do workspace, execute o source:
   ```bash
   source devel/setup.bash
   ```
4. Conecte ao robô e lance os nós de hardware:
   ```bash
   roslaunch synkar_robot_bringup bringup.launch
   ```
5. Para controle remoto via teleop:
   ```bash
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py
   ```
6. Para visualizar as informações da câmera RealSense no RVIZ:
   ```bash
   rviz
   ```


---

## Controle via Joystick (Controle Xbox)

Para ser controlado via joystick serial é necessário criar um novo nó. Siga os passos abaixo

1. Crie o pacote "**controle**"
   ```bash
   ~$ cd ~/catkin_ws/src
   ~$ catkin_create_pkg controle rospy std_msgs sensor_msgs joy teleop_twist_joy
   ```
2. Estrutura de pastas
   ```bash
   ~$ cd ~/catkin_ws/src/controle
   ~$ mkdir launch config scripts

   continua...
   




