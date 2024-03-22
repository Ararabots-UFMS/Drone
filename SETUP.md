# Configuração do Ambiente de Desenvolvimento

Essa configuração é feita considerando o sistema operacional Ubuntu, visto que alguns do programas não possuem versão para Windows e Mac.

Programas a serem instalados:

* Gazebo
* ROS/MAVROS
* QGroundControl
* BetaFlight

## Gazebo e ROS/MAVROS

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




## BetaFlight