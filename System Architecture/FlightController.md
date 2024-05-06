# Flight Controller (Somente)

![image](https://github.com/Ararabots-UFMS/Drone/assets/104502725/2832ae2f-d00d-49d3-a61c-312892657ae7)

O hardware é composto por:

- [Controlador de voo](https://docs.px4.io/main/en/flight_controller/) (executando o conjunto de pilha de voo PX4). Isso muitas vezes inclui IMUs internos, bússola e barômetro.
- [ESCs de motores](https://docs.px4.io/main/en/peripherals/esc_motors.html) conectados às saídas [PWM](https://docs.px4.io/main/en/peripherals/pwm_escs_and_servo.html), [DroneCAN](https://docs.px4.io/main/en/dronecan/escs.html) (DroneCAN permite comunicação bidirecional, não unidirecional como mostrado) ou algum outro barramento.
- Sensores ([GPS](https://docs.px4.io/main/en/gps_compass/), bússola, sensores de distância, barômetros, fluxo óptico, barômetros, transponders ADSB, etc.) conectados via I2C, SPI, CAN, UART, etc.
- [Câmera](https://docs.px4.io/main/en/camera/configuration) ou outra carga útil. Câmeras podem ser conectadas às saídas PWM ou via MAVLink.
- [Rádios de telemetria](https://docs.px4.io/main/en/telemetry/) para conexão com um computador/software em estação terrestre.
- [Sistema de Controle RC](https://docs.px4.io/main/en/getting_started/rc_transmitter_receiver.html) para controle manual.

O lado esquerdo do diagrama mostra a pilha de software, que está alinhada horizontalmente (aproximadamente) com as partes de hardware do diagrama.

- O computador da estação terrestre geralmente executa o QGroundControl (ou algum outro software de estação terrestre). Também pode executar software de robótica como MAVSDK ou ROS.
- A pilha de voo PX4 em execução no controlador de voo inclui drivers, módulos de comunicação, controladores, estimadores e outros módulos de middleware e sistema.


