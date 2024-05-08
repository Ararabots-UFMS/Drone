# Workspaces no ROS2

Este é um guia de como funcionam os workspaces do ROS2 e como trabalhar com eles.

## O que é um workspace no ROS2

Um workspace basicamente é um diretório que contém pacotes ROS2. 

Sempre que for utilizar o ROS é necessário fazer o source da instalação, para que os comandos estejam disponíveis na sessão atual do terminal

```bash
source /opt/ros/humble/setup.bash
```

(Caso sua shell padrao seja zsh, mude para setup.zsh)

Também é possível fazer o source de um "overlay", ou seja, sobrepor o workspace existente do ROS2 (seria o "underlay") com um workspace que você criou.

Dessa forma, é possível adicionar novos pacotes sem interferir com o workspace existente do ROS2 ("underlay"). O underlay deve conter todas as dependencias dos pacotes no overlay. Ao fazer source de um overlay, os pacotes presentes nele sobrescrevem os do underlay caso possuam a mesma definição. Também é possíovel ter diversas camadas de overlays e underlays, sendo que cada novo overlay contém os pacotes dos seus underlays.

![overlay-underlay](https://github.com/Ararabots-UFMS/Drone/tree/main/Tutoriais/images/overlay-underlay.png)

Na imagem acima o ROS2 é o underlay, e podemos fazer source do pacote "controle_voo" ou do "sensor_lidar" e utilizá-los em terminais diferentes sem algum problema, ou até mesmo no mesmo terminal. 

Já o pacote "Drone", para fazer build dele, é necessário fazer source do "controle_voo" e do "sensor_lidar", visto que ele depende deles, e ao fazer source do pacote "Drone" os outros dois já são incluidos automaticamente.