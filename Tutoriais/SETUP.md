# Configuração do Ambiente de Desenvolvimento

Essa configuração é feita considerando o sistema operacional Ubuntu, visto que alguns do programas não possuem versão para Windows e Mac.

Programas a serem instalados:

- Gazebo
- ROS2
- QGroundControl
- BetaFlight
- PX4
- Visual Studio Code
- Colcon

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

## PX4

Também precisamos instalar o PX4 (do fork Drone-PX4) para rodar simulações junto ao Gazebo.
Para instalar, utilize os seguintes comandos:

```shell
cd
mkdir PX4Autopilot
git clone https://github.com/Ararabots-UFMS/Drone-PX4.git ./PX4-Autopilot --recursive --recurse-submodules=
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
git fetch upstream
git fetch upstream --tags
make px4_sitl
```

Para comunicação com o ROS2, é necessário instalar algumas dependências:

```shell
pip install --user -U empy==3.3.4 pyros-genmsg setuptools
```

Além dessas, precisamos instalar o Micro XRCE-DDS Agent (**DEVE SER INSTALADO NO COMPANION COMPUTER**):

```shell
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

Para executar o Agent use:

```shell
MicroXRCEAgent udp4 -p 8888
```

## Gazebo

O Gazebo já é instalado durante a instalação do PX4

![image](https://github.com/Ararabots-UFMS/Drone/assets/104502725/d8bca06f-23c8-47ae-ac94-8f1bd5f11626)
Compatibilidade entre gazebo e ros

 
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

## Visual Studio Code

Faça o download do pacote debian do VS code [neste link](https://code.visualstudio.com/).

E instale com o seguinte comando:

```shell
sudo apt install ./<file>.deb
```
## Colcon

O colcon é a ferramenta utilizada para build dos workspaces do ROS2, para instalar basta executar os seguintes comandos:

```bash
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update
sudo apt install python3-colcon-common-extensions
```
