# Configuração do Ambiente de Desenvolvimento

Essa configuração é feita considerando o sistema operacional Ubuntu, visto que alguns do programas não possuem versão para Windows e Mac.

Programas a serem instalados:

* Gazebo
* ROS2
* QGroundControl
* BetaFlight

## ROS2

Para a instalação dos ROS2 primeiro precisamos adicionar o seu repositório ao sistema, com os seguintes comandos:

```shell
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Agora vamos instalar os pacotes do ROS2:

```shell
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
```

Para verificar se está funcionando corretamente, abra dois terminais e execute os seguintes comandos.

No terminal 1:
```shell
ros2 run demo_nodes_cpp talker
```

No terminal 2:
```shell
ros2 run demo_nodes_py listener
```

No terminal 1 serão enviadas mensagens que devem aparecer no terminal 2. 

## Gazebo

Para instalar o gazebo, primeiro é necessário instalar algumas dependências, com os seguintes comandos:

```shell
sudo apt-get update
sudo apt-get install lsb-release wget gnupg
```

E então instalar o Gazebo Ignition Fortress (Versão compatível com o ROS2 Humble)

```shell
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt-get install ignition-fortress
```

Para realizar a comunicação entre o ROS2 e o Gazebo precisamos instalar uma outra ferramenta:

```shell
sudo apt-get install ros-humble-ros-ign-bridge
```

## QGroundControl

Seguindo o tutorial de instalação no site oficial do programa.

Antes de instalar é necessário executar os seguintes comandos para permitir acesso às portas seriais do computador e instalar as bibliotecas necessárias:

```shell
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0 -y

```

Após executar os comandos, faça logoff e login novamente e faça o [download do arquivo](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#:~:text=Download%20QGroundControl.AppImage).

Feito o download, execute o seguinte comando:

```shell
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage
```

## BetaFlight

Para fazer o download do BetaFlight, basta [clicar aqui](https://github.com/betaflight/betaflight-configurator/releases/download/10.9.0/betaflight-configurator_10.9.0_amd64.deb).

Para instalar, execute:

```shell
sudo apt-get install ./betaflight-configurator_10.9.0_amd64.deb
```