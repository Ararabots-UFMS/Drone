# Drone

## Autonomous Drone Project

Baseado no PX4 source flight controller firmware. Utiliza ambientes simulados como (Gazebo), implementa modulos de mapeamento, localizcao, planejamento, controle e deteccao.

		Necessarios e utilizados:
		https://px4.io/
		https://www.ros.org/
		http://qgroundcontrol.com/
		https://mavsdk.mavlink.io/main/en/index.html
		https://mavlink.io/en/
		https://docs.qgroundcontrol.com/master/en/qgc-user-guide/
		https://camera-manager.dronecode.org/en/
		https://ardupilot.org/ardupilot/
  		https://clover.coex.tech/en/       ---     https://coex.tech/education
		https://www.ecalc.ch/xcoptercalc_mobile.php
		http://www.librepilot.org/site/index.html
		https://ieeexplore.ieee.org/Xplore/home.jsp

## Lista de Livros

- [Quan, Quan - Introdução ao Design e Controle de Multirrotores](https://www.springer.com/gp/book/9789811033810)
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;

Um livro muito completo em geral. Ele fornece uma visão geral detalhada de todos os tópicos relacionados a quadricópteros, desde o design do chassi até eletrônicos e controle, mergulhando profundamente em práticas e considerações técnicas e práticas de design. Ele fornece intuição por trás de cada assunto e não contém ambiguidades. A matemática pode parecer muito difícil para algumas pessoas lidarem, mas isso não é motivo de preocupação. Uma pessoa sem formação em engenharia mecânica ou elétrica pode facilmente lidar com isso com algum conhecimento de cálculo e geometria básica.


## Você não pode absorver tudo de uma vez

Faça pausas, anote possíveis coisas que você precisa revisar e assista-as depois ou de uma maneira diferente. Isso é muito para absorver e pausas são obrigatórias.

Alguns conceitos podem levar muito mais tempo para serem entendidos completamente. Então, se você não entender imediatamente, não se preocupe. Sugiro revisar algumas partes que pareceram difíceis para você após passar por esta lista de estudos uma vez.
## Foco
Existem muitas distrações que podem consumir tempo valioso. Foco e concentração são difíceis.

## Conhecimento Prévio

Aprenda a programar. Isso é necessário para entender como os diferentes algoritmos e técnicas que serão apresentados são realmente implementados. Você precisa desse conhecimento para poder entender o código-fonte das implementações populares de estruturas de dados, controladores de voo de código aberto e muito mais. Estruturas de dados básicas e complexidade algorítmica devem ser incluídas.

- [ ] **C, C++, Python, Qualquer coisa**
    - Informações disponíveis na internet são amplamente acessíveis e você pode encontrar muitas coisas pesquisando.
    - Posso recomendar começar com Python e progredir para C e C++.
    - Você pode aprender a programar em paralelo com este plano de estudos, mas as coisas serão muito mais difíceis e levarão mais tempo.
    - [ ] [Um plano de estudos completo de ciência da computação para se tornar um engenheiro de software](https://github.com/jwasham/coding-interview-university)

A próxima parte é necessária para poder entender a matemática por trás das coisas que vamos usar. Não é de forma alguma mais difícil em comparação com outras áreas de estudo. Aguenta firme e, no final, você ficará surpreso com o quão fácil é entender tudo. Este não é o único caminho para aprender essas coisas, é apenas o que eu seguiria. (Sim, eu gosto de palestras universitárias e artigos técnicos.)

- [ ] **Álgebra linear básica, cálculo, probabilidade e física elementar**
    - [ ] [Cálculo em Uma Variável - MIT OCW](https://ocw.mit.edu/courses/mathematics/18-01sc-single-variable-calculus-fall-2010/)
    - [ ] [Cálculo em Múltiplas Variáveis - MIT OCW](https://ocw.mit.edu/courses/mathematics/18-02sc-multivariable-calculus-fall-2010/)
    - [ ] [Álgebra Linear de Gilbert Strang - MIT OCW](https://ocw.mit.edu/courses/mathematics/18-06-linear-algebra-spring-2010/) __realmente não há melhor maneira de aprender isso no mundo__
    - [ ] [Introdução à Probabilidade e Estatística - MIT OCW](https://ocw.mit.edu/courses/mathematics/18-05-introduction-to-probability-and-statistics-spring-2014/)
    - [ ] [Processamento Digital de Sinais - Rensselaer Polytechnic Institute](https://www.youtube.com/playlist?list=PLuh62Q4Sv7BUSzx5Jr8Wrxxn-U10qG1et)
     

## Cursos

__Opcionais__

- Esses podem ser caros, mas aqui está uma lista dos mais populares. A maioria é barata ou gratuita.

- [Nanodegree de Carros Voadores](https://www.udacity.com/course/flying-car-nanodegree--nd787)
- [Robótica Aérea](https://www.coursera.org/learn/robotics-flight)

- [Crie um Drone de Código Aberto](https://www.udemy.com/course/make_a_drone/)
- [Primer de Programação de Drones](https://www.udemy.com/course/drone-programming-primer-for-software-development/)

- [IA Para Robótica](https://www.udacity.com/course/artificial-intelligence-for-robotics--cs373)
- [Visão Computacional](https://www.udacity.com/course/computer-vision-nanodegree--nd891)

- [Topografia com Drones](https://www.udemy.com/course/land-surveying-with-drones-fly-process-analyze-1/)
- [Modelagem e Mapeamento de Drones 2D & 3D Com Agisoft Metashape](https://www.udemy.com/course/2d-3d-drone-modeling-and-mapping-with-agisoft-metashape/)
- [Cursos de Drones no EDX: Aplicações e Agricultura](https://www.edx.org/learn/drones)

## Simulação e Controle

- Esta é a base dos rotores que será coberta primeiro. Estes poucos recursos farão você entender o que um drone precisa para voar mal, bem, com a ajuda de motores de autonomia extras ou com a ajuda de um piloto.
- Após esta parte, os detalhes de cada subsistema serão investigados minuciosamente.

- [ ] [Palestras Técnicas do Matlab - Compreendendo Sistemas de Controle](https://www.mathworks.com/videos/series/understanding-control-systems-123420.html)
- [ ] [Palestras Técnicas do Matlab - Simulação e Controle de Drones](https://www.mathworks.com/videos/drone-simulation-and-control-part-1-setting-up-the-control-problem-1539323440930.html)
- [ ] [Introdução à Simulação de 6 Graus de Liberdade de Veículos Aéreos (pdf)](http://avionics.nau.edu.ua/files/doc/VisSim.doc/6dof.pdf)
- [ ] [Dinâmica, Simulação e Controle de Quadricópteros (artigo)](http://andrew.gibiansky.com/downloads/pdf/Quadcopter%20Dynamics,%20Simulation,%20and%20Control.pdf)
- [ ] [PID, LQR e LQR-PID em uma plataforma de quadricóptero (artigo)](https://www.researchgate.net/publication/261212676_PID_LQR_and_LQR-PID_on_a_quadcopter_platform)
- [ ] [Betaflight: Guia de Ajuste de PID](https://www.youtube.com/watch?v=27lMKi2inpk)
- [ ] [Pixhawk: Sintonização Automática de PID](https://www.youtube.com/watch?v=DbcZCql1UlE)

__Neste ponto, você pode se perguntar: Isso é apenas para quatro rotores. Não se preocupe, os extras são apenas para ter resiliência. No futuro, isso será ampliado com mais tipos de aeronaves de rotores, como submarinos, drones VTOL e asas.__



## Teoria de Controle

- Agora que você tem uma compreensão básica de como vai fazer as coisas voarem, é hora de dar uma olhada mais aprofundada nos conceitos matemáticos por trás do controle de sistemas.

- [ ] [Palestras Técnicas do Matlab - Espaço de Estados](https://www.mathworks.com/videos/series/state-space.html)
- [ ] [Palestras Técnicas do Matlab - Sistemas de Controle na Prática](https://www.mathworks.com/videos/series/control-systems-in-practice.html)
- [ ] [Palestras Técnicas do Matlab - Compreendendo o Controle PID](https://www.mathworks.com/videos/series/understanding-pid-control.html)



## Sensores e Estimação de Estado

- Para fazer a teoria de controle funcionar, precisamos obter uma estimativa de estado de melhor esforço observando nosso sistema de aeronaves rotativas.

- [ ] Filtros de Kalman
    - [ ] [Palestras Técnicas do Matlab - Compreendendo os Filtros de Kalman](https://www.mathworks.com/videos/series/understanding-kalman-filters.html)
    - [ ] [(ou) Michel van Biezen - Filtros de Kalman](https://www.youtube.com/watch?v=CaCcOwJPytQ)
- [ ] [Filtros de Partículas](https://www.youtube.com/watch?v=lzN18y_z6HQ)
- [ ] [(opcional) Michel van Biezen - Como o GPS funciona](https://www.youtube.com/watch?v=16xHIBmul_o&list=PLX2gX-ftPVXXGdn_8m2HCIJS7CfKMCwol)
- [ ] Projeções de Mapa
    - [ ] [Projeções de Mapa Explicadas](https://www.youtube.com/watch?v=wlfLW1j05Dg)
    - [ ] [Projeção Equiretangular (site)](https://en.wikipedia.org/wiki/Equirectangular_projection)
- [ ] Unidades de Medição de Inércia
    - [ ] [Como Funcionam as IMUs](https://www.youtube.com/watch?v=eqZgxR6eRjo)
    - [ ] [Como Implementar uma IMU](https://www.youtube.com/watch?v=T9jXoG0QYIA)
    - [ ] [Montagem Suave e Vibrações](https://www.youtube.com/watch?v=zdE1BidMwNU)
- [ ] Câmeras e Sensores Ópticos
    - [ ] [Como Funciona uma Câmera](https://www.youtube.com/watch?v=qS1FmgPVLqw)
    - [ ] [Obturador Rolante e Global](https://www.youtube.com/watch?v=DG4OjpD3Zow)
    - [ ] Fluxo Óptico
        - [ ] [Computerphile - Fluxo Óptico](https://www.youtube.com/watch?v=5AUypv5BNbI)
        - [ ] [Computerphile - Soluções de Fluxo Óptico](https://www.youtube.com/watch?v=4v_keMNROv4)
    - [ ] Percepção de Profundidade
        - [Como o Kinect funciona em 2 minutos](https://www.youtube.com/watch?v=uq9SEJxZiUg)
        - [Noções básicas de 3D estereoscópico](https://www.youtube.com/watch?v=1MXNRrHLuWk)
        - [Como funciona um LiDAR](https://www.youtube.com/watch?v=EYbhNSUnIdU)
        - [O problema da correspondência](https://www.youtube.com/watch?v=VZNN1OGoqr8)]
    - [ ] [PX4: Controle de Gimbal (site)](https://dev.px4.io/v1.9.0/en/advanced/gimbal_control.html)
    - [ ] [MAVLink: Protocolo de Gimbal (site)](https://mavlink.io/en/services/gimbal.html)
- [ ] [Sensor de Distância Ultrassônico](https://www.youtube.com/watch?v=6F1B_N6LuKw)
- [ ] Correções e Calibração
    - [ ] [O que é Calibração de Sensor e Por que é Importante?](https://www.youtube.com/watch?v=n_lZCIA25aI)
    - [ ] [Filtro Passa-Baixa (site)](https://www.dsprelated.com/freebooks/filters/Simplest_Lowpass_Filter_I.html)
    - [ ] [Taxas de Amostragem para Sensores Analógicos (site)](https://www.embedded.com/sampling-rates-for-analog-sensors/)
    - [ ] [Reconstrução de Sinal](https://www.youtube.com/watch?v=rmDg3eVWT8E)
- [ ] Protocolos de Comunicação
    - [ ] [Compreendendo o Barramento I2C (pdf)](https://www.ti.com/lit/an/slva704/slva704.pdf)
    - [ ] [Como o I2C Funciona](https://www.youtube.com/watch?v=6IAkYpmA1DQ)
    - [ ] [UART (pdf)](https://www.ti.com/lit/ug/sprugp1/sprugp1.pdf)
    - [ ] [Como a UART Funciona](https://www.youtube.com/watch?v=V6m2skVlsQI)
    - [ ] [SPI para Iniciantes](https://www.youtube.com/watch?v=ba0SQwjTQfw)
    - [ ] [Conversores Analógico-Digitais](https://www.youtube.com/watch?v=EnfjYwe2A0w)
    - [ ] [SBUS e IBUS](https://www.youtube.com/watch?v=N2nnI72bmj4)
    - [ ] [Compreendendo a Modulação por Largura de Pulso](https://www.youtube.com/watch?v=YfV-vYT3yfQ)
    - [ ] [MAVLink](https://mavlink.io/)
    - [ ] [Guia completo do XBee](https://www.youtube.com/watch?v=odekkumB3WQ)
     

## SLAM

- O subsistema de sensores está completo. Agora podemos prosseguir para construir um mapa do ambiente, bem como descobrir onde nosso UAV está localizado no mundo.

- [ ] [Quadtrees e Octrees (site)](https://www.i-programmer.info/programming/theory/1679-quadtrees-and-octrees.html)
- [ ] [Octomap: Uma Estrutura de Mapeamento 3D Probabilística Eficiente Baseada em Octrees (site)](https://octomap.github.io/)
- [ ] [EKF SLAM - Cyrill Stachniss](https://www.youtube.com/watch?v=XeWG5D71gC0)
- [ ] [Visualização de Dados LiDAR](https://www.youtube.com/watch?v=nXlqv_k4P8Q)
- [ ] [Condução Autônoma: Localização e Mapeamento com Aprendizado Profundo](https://www.youtube.com/watch?v=xgI3vgnHQ9U)

## Planejamento de Trajetória

- Após uma localização adequada, estamos prontos para planejar a trajetória ou missão, para fazer com que o UAV complete um curso de trajetória predefinido junto com ações

- [ ] [Planejamento de Trajetória Robótica: RTT, RTT* (artigo)](https://medium.com/@theclassytim/robotic-path-planning-rrt-and-rrt-212319121378)
- [ ] [3DVFH+: Esquiva de Obstáculos Tridimensional em Tempo Real Usando um Octomap (artigo)](https://www.researchgate.net/publication/269872013_3DVFH_Real-Time_Three-Dimensional_Obstacle_Avoidance_Using_an_Octomap)
- [ ] [QGroundControl: Planejamento de Missão (site)](https://docs.qgroundcontrol.com/en/PlanView/PlanView.html)
- [ ] [ArduPilot: Modo ZigZag (site)](http://ardupilot.org/copter/docs/zigzag-mode.html)
- [ ] [ArduPilot: Lançamento Automático (site)](http://ardupilot.org/plane/docs/automatic-takeoff.html)

## Mecatrônica

- [ ] [Como funcionam os motores brushless e como controlá-los](https://www.youtube.com/watch?v=uOQk8SJso6Q)
- [ ] [Vantagens e Desvantagens de Motores Brushed e Brushless](https://www.youtube.com/watch?v=Y7nQI2xM2as)
- [ ] [Como escolher o motor brushless certo para seu drone](https://www.youtube.com/watch?v=Ry6JJPgrfVA)
- [ ] [Trem de Pouso Retrátil Lento para Drones](https://www.youtube.com/watch?v=VWCpLyIXwz0)
- [ ] [Drone com Braços de Robô Duplos](https://www.youtube.com/watch?v=T6kaU2sgPqo)




## Software Existente de Drones

- Para seu próprio interesse. Dê uma olhada no código-fonte e no rastreador de problemas, talvez participe das chamadas de desenvolvimento semanais.

- Autopilotos:
- [ ] [PX4](https://github.com/PX4/Firmware)
- [ ] [Ardupilot](http://ardupilot.org/)
- [ ] [BetaFlight](https://betaflight.com/)
- [ ] [Cleanflight](http://cleanflight.com/)
- [ ] [INAV](https://github.com/inavFlight/inav/wiki)

- SDKs:
- [ ] [MAVSDK](http://mavsdk.io/)
- [ ] [Ferramentas de Desenvolvimento Dronekit](https://dronekit.io/)
- [ ] [SDKs Parrot](https://developer.parrot.com/)
- [ ] [SDKs DJI](https://developer.dji.com/)
- [ ] [Habilidades Skydio](https://github.com/Skydio/skydio-skills)

- Estações de Controle em Terra:
- [ ] [QGroundControl](http://qgroundcontrol.com/)

- Simuladores:
- [ ] [Gazebo](http://gazebosim.org/)
- [ ] [Airsim](https://microsoft.github.io/AirSim/)

- Middleware:
- [ ] [Sistema Operacional de Robô](https://www.ros.org/)

- Protocolos:
- [ ] [UAVCAN](https://uavcan.org/)
- [ ] [MAVLink](https://mavlink.io/)
- [ ] [Betaflight: Firmware ESC Aberto](https://github.com/betaflight/betaflight-esc)
- [ ] [BLHeli](https://github.com/bitdump/BLHeli)

- Ferramentas de Design:
- [ ] [Avaliação de Voo](https://www.flyeval.com/) - Projeto e avaliação de estrutura aérea baseada em regras e restrições

## Hardware Existente de Drones

- Para o seu próprio interesse também.

- [ ] [Controladores Pixhawk](https://pixhawk.org/#autopilots)
- [ ] [Betaflight F4](https://www.getfpv.com/betaflight-f4-flight-controller.html)
- [ ] [Kakute F7](http://www.holybro.com/product/kakute-f7/)
- [ ] [Kiss FC](https://www.flyduino.net/en_US/shop/product/pr1872-kiss-fc-32bit-flight-controller-v1-03-2686)
- [ ] [Kit de Propulsão Zubax UAVCAN](https://shop.zubax.com/products/uav-propulsion-kit)
- [ ] [GPS NEO-M8N](https://www.getfpv.com/pixhawk-4-autopilot-and-neo-m8n-gps-pm07-combo.html)
- [ ] [Skydio](https://www.skydio.com/)
- [ ] [Intel Aero RTF](https://click.intel.com/intel-aero-ready-to-fly-drone-2812.html)
- [ ] [QAV 250 RTF](https://www.getfpv.com/qav250-mini-fpv-quadcopter-rtf-carbon-fiber-edition.html)
- [ ] [Intel Realsense](https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html)

## O Controlador de Voo

- [ ] [Betaflight: Diagramas de Arquitetura de Software](https://github.com/betaflight/betaflight/issues/7112)
- [ ] [Betaflight: Alvos de Placas Unificadas](https://github.com/betaflight/betaflight/wiki/Unified-Targets)
- [ ] [Pixhawk: Visão Arquitetural](https://dev.px4.io/v1.9.0/en/concept/architecture.html)
- [ ] [ArduPilot: Modos de Voo](http://ardupilot.org/copter/docs/flight-modes.html)
- [ ] [Pixhawk: Revisão de Voo](https://docs.px4.io/v1.9.0/en/log/flight_review.html)

## Construindo um Quadricóptero FPV de Corrida

- Para fazer isso, você precisa encontrar um conjunto de hardware necessário que seja compatível, gravar e configurar o firmware do controlador de voo desejado.

- [ ] **Hardware Mínimo**
    - [ ] Estrutura. Para corridas/controle totalmente manual, geralmente de fibra de carbono, mas você pode até mesmo imprimir em 3D ou esculpir em madeira. Considere 2 dimensões: tamanho do conjunto do controlador de voo e tamanho da hélice. A maioria das hélices comuns teria cerca de 5".
    - [ ] Controlador de Voo. Você já sabe o suficiente para escolher um. Eu usaria o Betaflight.
    - [ ] ESCs. Você pode optar por um ESC 4 em 1 ou uma placa combo all-in-one. Cada um tem vantagens e desvantagens, pesquise sobre isso. Mas primeiro, verifique sua bateria. Deve ser capaz de lidar com a corrente de pico.
    - [ ] PDB. Você precisa de uma placa de distribuição de energia. Estas são relativamente pequenas e simples. Você pode encontrar combos __PDB+ESC+FC__ que incluem redução de tensão da bateria e tudo mais. Mas primeiro, verifique sua bateria.
    - [ ] Bateria. LiPo. No momento desta escrita, você pode ir de 4s a 6s. O preço depende da capacidade, classificação de descarga e contagem de células.
    - [ ] Motores Brushless. Você sabe, para fazer a coisa se mover.
    - [ ] Transceptor RC. Deve ser compatível com seu controlador RC e seu protocolo deve ser compatível com sua placa FC. Tipicamente 2.4GHz.
    - [ ] Controlador RC. Flysky, Taranis, há muitos para listar.
    - Após configurar o firmware, você deve ser capaz de voar com isso, mas há várias coisas faltando.
- [ ] **Hardware Opcional**
    - [ ] Câmera. Não é do tipo que você está acostumado. Esta envia vídeo (analógico) PAL/VTSC e deve suportar [OSD](https://oscarliang.com/betaflight-osd/)
    - [ ] VTX. Significa transmissor de vídeo. Isso é usado para transmitir o vídeo geralmente em 5.8GHz. Pesquise quantos mW você precisa.
    - [ ] Óculos FPV. Para visualizar o vídeo. Deve ter antena compatível com o VTX. Módulos de diversidade que escolhem dinamicamente a melhor fonte de sinal de uma antena onidirecional e uma direcional são melhores.
    - [ ] Antenas Aftermarket. Para melhor recepção.
    - [ ] Buzzer. Apita se você perder.
    - [ ] Carregador de Bateria com Balanceamento. Você não pode usar baterias descarregadas.
    - [ ] Fonte de Alimentação do Carregador de Bateria com Balanceamento. Se seu carregador não vier com um. (A maioria dos IMAX B6s não vem, por exemplo)
    - [ ] Caixa 3D Impressa para GoPro Session
    - [ ] Fita Isolante
    - [ ] Abraçadeiras
    - [ ] LEDs para a parte inferior da aeronave
    - [ ] Fitas de Velcro para Bateria

- [ ] **Software e Configuração Adicional**
    - [ ] Registro Blackbox. Informações importantes sobre o seu voo. Pode ajudar na análise pós-voo.
    - [ ] Mapeamento de Canais RC
    - [ ] Certifique-se de que os ESCs girem nas direções corretas. Você pode mudar isso no software do ESC ou do controlador de voo.
    - [ ] Emparelhe o Transceptor RC com seu Controlador RC.
    - [ ] Ajuste de PID e Taxas
    - [ ] Mantenha um backup da configuração junto com o número exato do firmware para que você não se esqueça.
     
Pilotos experientes de corridas recomendam começar voando no modo 'acro'/'freestyle' desde o início. Não faça isso com hardware de drone real, a probabilidade de colisão dentro de 2 segundos é quase 100%. Dedique seu tempo primeiro a um simulador. Existem muitos disponíveis nas plataformas de jogos mais populares, como Steam. Por exemplo: Liftoff, DRL Sim, etc.

Voar no modo 'angle'/'aided' é muito mais fácil e é o mesmo que voar com seu drone DJI/Parrot/etc típico. Você pode voar dessa maneira mesmo sem óculos.

## Construção de Rotorcraft Totalmente Autônomo

- Construir um Rotorcraft Totalmente Autônomo é muito mais difícil e requer mais recursos do que construir um com assistência computacional. Você precisa de uma calibração melhor porque não há um piloto voando diretamente, apenas supervisionando.

- **Hardware Básico**. Básico no sentido de que você precisa pelo menos disso para voar autonomamente. Talvez você possa deixar o GPS de fora, mas você precisa pelo menos de fluxo óptico para estabilização exata.
    - [ ] Controlador de Voo. Prefira um pré-ajustado e pré-construído que seja resistente e robusto o suficiente. O [Pixhawk 4](https://docs.px4.io/master/en/flight_controller/pixhawk4.html) é uma opção muito boa, com duplos IMUs, uma tonelada de sensores, conectores padronizados e uma variedade de opções de conectividade.
    - [ ] GPS
    - [ ] e/ou Câmera(s) Montada(s) na Parte Inferior. Um exemplo de um drone que possui câmeras na parte inferior, mas não GPS, é o da série de palestras sobre tecnologia MATLAB.
    - [ ] Estrutura
    - [ ] Hélices
    - [ ] Motores Brushless
    - [ ] Placa de Distribuição de Energia
    - [ ] Bateria LiPo
    - [ ] Conectividade
        - [ ] Alguma forma de enviar dados de controle de missão e receber informações
        - [ ] Controlador RC para sobreposição do piloto à trajetória

- [ ] **Hardware Opcional**
    - **Esquiva Básica de Obstáculos**
        - [ ] Câmera Montada na Frente (ou RGB-D ou ultrassônico de longo alcance para drones de movimento lento)
        - [ ] Co-processador básico se o firmware do controlador de voo não conseguir lidar com a esquiva de obstáculos por conta própria
    - **Pouso Automático e Manual**
        - [ ] Sensor ultrassônico montado na parte inferior
        - [ ] e/ou câmera(s) montada(s) na parte inferior - são necessárias duas câmeras para a profundidade, a fim de serem mais confiáveis do que o fluxo óptico
    - **Transporte de Carga**
        - [ ] Algum tipo de [ímã](https://kb.zubax.com/display/MAINKB/OpenGrab+EPM+v3) ou servo para soltar cargas
    - **Operações Avançadas de Câmera**
        - [ ] Gimbal rotativo. Para não acoplar o campo de visão com o movimento do drone.
        - [ ] Câmera RGB-D (para melhor SLAM)
        - [ ] LiDAR (também para SLAM)
    - [ ] Trem de Pouso Retrátil
    - **Conectividade Extra**
        - [ ] WiFi
        - [ ] LoRa
        - [ ] E-3G-4G-5G
        - [ ] Bluetooth

- **Placas de Computação Co-processadoras Úteis**
    - [Raspberry Pi](https://www.raspberrypi.org/)
    - [ODroid](https://www.hardkernel.com/)

    - [nVidia Jetson](https://www.nvidia.com/en-us/autonomous-machines/jetson-store/)
    - [Acelerador Coral USB ML](https://coral.withgoogle.com/products/accelerator/)
    - [Intel Neural Compute USB Stick](https://software.intel.com/en-us/neural-compute-stick)

Normalmente, você começa com os requisitos do 'trabalho', por exemplo, mapeamento, pulverização de culturas, inspeção e, em seguida, decide qual hardware é capaz de fazer esse trabalho em um cenário de custo-benefício. Configure tudo e você estará pronto para ir.

## Depois de Concluir

Parabéns!

Continue aprendendo.

Você nunca está realmente pronto. Mas ainda assim, bom trabalho :).
- [ ] [Visão Noturna Digital: Insights](https://www.youtube.com/watch?v=CFDNEjJ0cME)
- [ ] [Como Fazer uma Câmera de Visão Noturna Infravermelha a Partir de uma Câmera Digital Comum](https://www.youtube.com/watch?v=qRCz6n_cqHM)

- [ ] [O Robô Aéreo Reconfigurável: Modelagem e Controle](https://www.youtube.com/watch?v=yarPEP8Kcwk)
- [ ] [Skydio: Palestra de Convidado de Adam Bry CS287](https://www.youtube.com/watch?v=ZI66eq7Nn1E)

- [ ] [Exploração Autônoma Dentro de Corredores de Prédios](https://www.youtube.com/watch?v=H7WpBhPbvSI)
- [ ] [Autoimplantação Autônoma de Marcadores de Comunicação](https://www.youtube.com/watch?v=-bDDzUFlJGE)

- [ ] [Próxima Geração de Nano Drones Silenciosos do Exército](https://www.youtube.com/watch?v=d5TdbMu8xc4)

## Uso de Drones na Indústria

## Outros Tipos de Veículos

- **Asas**
- **VTOL**
- **Subaquático**

## Tópicos Mais Avançados

- [ ] Enxames de UAVs

- [ ] Reabastecimento Automático e Retomada de Missão

- [ ] Aprendizado Profundo
    - [ ] [Especialização em Aprendizado Profundo de Andrew Ng](https://www.coursera.org/specializations/deep-learning)
    - [ ] [Cursos Fast AI](https://www.fast.ai/)

Agora que você chegou tão longe, aqui estão os últimos recursos aos quais posso direcioná-lo. A partir deste ponto, você deve ter um bom entendimento de como os drones funcionam e como ter uma percepção digital do ambiente. As últimas partes são exatamente algumas peças que faltam e como juntar tudo:

- [ ] Rastreamento Eficiente e Preciso de Objetos Através de Vídeo
    - [ ] [Filtros de Correlação Kernelizados](http://www.robots.ox.ac.uk/~joao/circulant/). Esta é a abordagem eficiente #1. Uma etapa de treinamento muito simples com uma detecção muito simples e uma etapa de atualização online.
    - [ ] [Aprendizado de Representação de Ponta a Ponta para Rastreamento Baseado em Filtros de Correlação](http://www.robots.ox.ac.uk/~vedaldi//assets/pubs/bertinetto17end-to-end.pdf). Esta é a etapa #2, sacrificando algum poder computacional e complexidade para obter um rastreamento mais robusto e invariante à transformação e oclusão.
- [ ] Computações Leves de Redes Neurais Profundas para Plataformas Móveis e Embarcadas
    - [ ] [MobileNets: Redes Neurais Convolucionais Eficientes para Aplicações de Visão Móvel](https://arxiv.org/pdf/1704.04861.pdf). Há também uma versão V2 do MobileNets.
    - [ ] [Intel OpenVINO](https://medium.com/@zarkopafilis/introduction-to-intel-openvino-in-5-minutes-b21f59e5fd4?source=friends_link&sk=2112ee80b16fa7da480d55ed1ed7602e). Otimizando modelos de aprendizado profundo para a borda. A nVidia e outras empresas fornecem suas próprias ferramentas, mas todas trabalham em uma poda semelhante e reduzem os neurônios enquanto mantêm métricas de desempenho suficientemente altas.
- [ ] [Em Direção a uma Plataforma Robusta de Cinematografia Aérea: Localização e Rastreamento de Alvos em Ambientes Não Estruturados](https://arxiv.org/pdf/1904.02319.pdf). AKA o paper da Skydio. Os autores deste paper publicaram muitos trabalhos sobre o assunto e provavelmente continuarão a fazê-lo. Sugiro que você os procure no Google Scholar.

## Outros Interessantes

- [Podcasts de IA de Lex Fridman](https://lexfridman.com/ai/)
- [2 Minute Papers](https://www.youtube.com/channel/UCbfYPyITQ-7l4upoX8nvctg)

Por fim, espero que tenha gostado de ler e aprender com este simples arquivo de texto tanto quanto eu gostei de pesquisar e escrever sobre ele.
