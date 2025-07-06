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

1. **ROS Noetic**Siga o guia oficial:
   ```bash
   # configuração das chaves e repositórios
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
   sudo apt update
   sudo apt install ros-noetic-desktop-full
   ```
2. **Pacote de teleop**Para controle via teclado:
   ```bash
   sudo apt install ros-noetic-teleop-twist-keyboard
   ```
3. **Pacote RealSense**
   Para visualização da câmera Intel RealSense:
   ```bash
   sudo apt install ros-noetic-realsense2-*
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

## Contribuição

1. Fork este repositório.
2. Crie uma _branch_ com sua feature:
   ```bash
   git checkout -b feature/nova-funcionalidade
   ```
3. Faça _commit_ das alterações:
   ```bash
   git commit -m "Descrição da funcionalidade"
   ```
4. Envie para o repositório remoto:
   ```bash
   git push origin feature/nova-funcionalidade
   ```
5. Abra um Pull Request.
