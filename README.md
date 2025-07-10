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

Para controlar o robô via joystick serial é necessário criar um novo nó. Siga os passos abaixo:

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


   ---
   
## Acessando a câmera D435

   Esses processos foram realizados no:

      Ubuntu 20.04

      ROS Noetic

      A câmera RealSense conectada via USB

      Terminal com internet

   1. Conecte a camera do robo via serial e verifique se ela foi reconhecida pelo sitema
      ```bash
      ~$ lsusb
      ```
      Você deve ver algo como:
      "Bus 002 Device 004: ID 8086:0b07 Intel Corp. Intel(R) RealSense(TM) Depth Camera"

   2. Para verificar os dispositivos criados:
      ```bash
      ls /dev/video*
      ```
      Você verá /dev/video0, /dev/video1, etc. (a D435 pode criar múltiplos dispositivos para RGB e IR).

   3. Corrigir repositórios da Intel RealSense

      Remover entradas duplicadas ou antigas

      Abra o arquivo principal:
      ```bash
      ~$ sudo nano /etc/apt/sources.list
      ```
   4. Procure e comente qualquer linha que contenha:
      
      http://realsense.intel.com/Debian/apt-repo
      
      **Salve com Ctrl + O, tecle Enter, e feche com Ctrl + X.**
      
   5. Edite o arquivo do repositório da RealSense:
      ```bash
      ~$ sudo nano /etc/apt/sources.list.d/realsense-public.list
      ```
      - Apague tudo e insira apenas esta linha:
      
         deb https://librealsense.intel.com/Debian/apt-repo focal main
        
      **Salve com Ctrl + O, tecle Enter, e feche com Ctrl + X.**
      
   6. Adicionar a chave pública oficial da Intel
      ```bash
      ~$ curl -sSf https://librealsense.intel.com/Debian/librealsense.public.key | sudo apt-key add -
      ```
   7. Atualizar e instalar pacotes
      ```bash
      ~$ sudo apt update
      ~$ sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev
      ~$ sudo apt install ros-noetic-realsense2-camera
      ```
   8. Instalando os drivers
      ```bash
      ~$ sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6B0FC61
      ~$ sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo $(lsb_release -cs) main"
      ~$ sudo apt update
      ~$ sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg7
      ```      
   9. Testar a câmera sem ROS (modo direto)
      - Com a câmera conectada, digite:
      ```bash
      ~$ realsense-viewer

      Você deve ver vídeo ao vivo e sensor de profundidade. Se isso funcionar, a câmera está OK via USB.

   **10. A partir daqui, o source é necessário em cada terminal novo que será rodado as funções do ROS**
      ```bash
      ~$ cd catkin_ws
      ~$ source devel/setup.bash
      ```      
   11. Rodar no ROS (com rs_camera.launch)
      - Inicie o ROS core:
      ```bash
      ~$ roscore
      ```      
      - Abra outro terminal e execute o driver da câmera:
      ```bash
      ~$ roslaunch realsense2_camera rs_camera.launch
      ```      
      **- Isso inicia os tópicos com vídeo, profundidade, infravermelho, etc.**
   
   12. Visualizar no rqt_image_view:
      - Em novo terminal:
      ```bash
      ~$ rqt_image_view
      ```
      Clique no menu suspenso e selecione:
      
      camera/color/image_raw → imagem da câmera RGB      
      camera/depth/image_rect_raw → imagem de profundidade

      Existem outras opções de imagens
      
   13. Visualizar no RViz
      - Em novo terminal:
      ```bash
      ~$ rviz
      ```
      
      - No painel esquerdo:
      
      **Clique em "Add"**
      
      **Selecione "Image"**
      
      **No campo "Image Topic", selecione: camera --> color --> image_raw --> Ok**
      
      Você verá a imagem da câmera dentro do RViz.

   14. Como fechar o RViz corretamente
      
      Ao iniciar o RViz duas abas irão se abrir, uma é o ambiente grafico do RViz e a outra é um aviso do fim de suporte do ROS.
      Se você tentar rodar o RViz sem fechar ou dar "ok" nessa aba em alguns casos pode crashar.
      Se você tentou fechar o rviz e ele não liberou o terminal, pode encerrá-lo com segurança (as vezes isso pode acontecer por causa da aba do aviso):
      Se estiver travado:
      
      Use Ctrl + C no terminal que o lançou
      
      Se não sair, abra outro terminal e digite:
      ```bash
      ~$ pkill rviz
      
      Isso força o fechamento sem comprometer o ROS.
   




