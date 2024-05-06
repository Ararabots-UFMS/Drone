# FC (Controlador de Voo) e Computador 

![image](https://github.com/Ararabots-UFMS/Drone/assets/104502725/b3c3ea22-6c63-4a36-a2fc-b323581fe6cd)


O controlador de voo executa a flight stack PX4 normal, enquanto um computador companheiro fornece recursos avançados como [evitação de objetos](https://docs.px4.io/main/en/computer_vision/obstacle_avoidance.html) e [prevenção de colisões](https://docs.px4.io/main/en/computer_vision/collision_prevention.html).
Os dois sistemas estão conectados usando uma ligação serial rápida ou IP e geralmente comunicam-se usando o [protocolo MAVLink](https://mavlink.io/en/).

As comunicações com as estações terrestres e a nuvem geralmente são roteadas através do computador companheiro (por exemplo, usando o [Roteador MAVLink](https://github.com/mavlink-router/mavlink-router) (da Intel)).
Os sistemas PX4 normalmente executam um sistema operacional Linux no computador companheiro (porque o projeto [PX4/PX4-Avoidance](https://github.com/PX4/PX4-Avoidance) fornece bibliotecas de evitação baseadas em ROS projetadas para Linux).

O Linux é uma plataforma muito melhor para o desenvolvimento de software "geral" do que o NuttX; existem muito mais desenvolvedores de Linux e uma quantidade significativa de software útil já foi escrito (por exemplo, para visão computacional, comunicações, integrações de nuvem, drivers de hardware). Os computadores companheiros às vezes executam o Android pelo mesmo motivo.
