# [Diagrama de Controle](https://docs.px4.io/main/en/flight_stack/controller_diagrams.html)

[!! notações !!](https://docs.px4.io/main/en/contribute/notation.html)

# Arquitetura de Controle de Multirrotores

![image](https://github.com/Ararabots-UFMS/Drone/assets/104502725/ade9f846-645e-4acc-950e-1e9519eb3f37)

Diagrama de Controle de MC (Multicopter Controller)

- Esta é uma arquitetura de controle em cascata padrão.
- Os controladores são uma mistura de controladores P e PID.
- As estimativas vêm do EKF2.
- Dependendo do modo, o loop externo (de posição) é ignorado (mostrado como um multiplexador após o loop externo). O loop de posição só é usado quando se está mantendo a posição ou quando a velocidade solicitada em um eixo é nula.

# Controlador de Taxa Angular de Multirrotores

![image](https://github.com/Ararabots-UFMS/Drone/assets/104502725/f9fc864a-1d85-43bb-8233-4bfbc5a57fba)

Diagrama de Controle de Taxa de MC (Multicopter Rate Control)

- Controlador K-PID. Consulte Controlador de Taxa para mais informações.
- A autoridade integral é limitada para evitar wind up.
- As saídas são limitadas (no módulo de alocação de controle), geralmente em -1 e 1.
- Um Filtro Passa-Baixas (LPF) é usado no caminho derivativo para reduzir o ruído (o driver do giroscópio fornece uma derivada filtrada para o controlador).


INFO:
![image](https://github.com/Ararabots-UFMS/Drone/assets/104502725/10fb8f5a-3719-4d2b-80a3-6ab04eaa0efd)
O pipeline do IMU é: dados do giroscópio > aplicar parâmetros de calibração > remover bias estimado > filtro notch (IMU_GYRO_NF0_BW e IMU_GYRO_NF0_FRQ) > filtro passa-baixas (IMU_GYRO_CUTOFF) > velocidade_angular_do_veículo (taxa angular filtrada usada pelos controladores P e I) > derivada -> filtro passa-baixas (IMU_DGYRO_CUTOFF) > aceleração_angular_do_veículo (aceleração angular filtrada usada pelo controlador D)

# Controlador de Atitude de Multirrotores
![image](https://github.com/Ararabots-UFMS/Drone/assets/104502725/48cfd68c-8e54-4a6a-910b-097080ca839a)

# Conversão de Aceleração para Empuxo e Ponto de Referência de Atitude de Multirrotores
Os pontos de referência de aceleração gerados pelo controlador de velocidade serão convertidos em pontos de referência de empuxo e atitude.

- Os pontos de referência de aceleração convertidos serão saturados e priorizados em empuxo vertical e horizontal.
- A saturação de empuxo é feita após o cálculo do empuxo correspondente:
    - Calcula-se o empuxo vertical necessário (thrust_z).
    - Satura-se thrust_z com MPC_THR_MAX.
    - Satura-se thrust_xy com (MPC_THR_MAX^2 - thrust_z^2)^0.5.

Os detalhes de implementação podem ser encontrados em PositionControl.cpp e ControlMath.cpp.

# Diagrama Controle de Velocidade
![image](https://github.com/Ararabots-UFMS/Drone/assets/104502725/61a905fb-555a-46db-8f0b-f6d467389ac8)
Controlador PID para estabilizar a velocidade. Comanda uma aceleração.

- O integrador inclui um anti-reset windup (ARW) usando um método de limitação.
- A aceleração comandada NÃO é saturada - uma saturação será aplicada aos pontos de referência de empuxo convertidos em combinação com o ângulo de inclinação máximo.
- Ganhos horizontais definidos via parâmetro MPC_XY_VEL_P_ACC, MPC_XY_VEL_I_ACC e MPC_XY_VEL_D_ACC.
- Ganhos verticais definidos via parâmetro MPC_Z_VEL_P_ACC, MPC_Z_VEL_I_ACC e MPC_Z_VEL_D_ACC.
  
# Diagrama Controle de Posicao 
![image](https://github.com/Ararabots-UFMS/Drone/assets/104502725/004b22f7-4d4c-442f-b74c-4be6d67395c1)

Controlador P simples que comanda uma velocidade.

- A velocidade comandada é saturada para manter a velocidade dentro de certos limites. Consulte o parâmetro MPC_XY_VEL_MAX. Este parâmetro define a velocidade horizontal máxima possível. Isso difere da velocidade máxima desejada MPC_XY_CRUISE (modos autônomos) e MPC_VEL_MANUAL (modo de controle de posição manual).

- Ganho P horizontal definido via parâmetro MPC_XY_P.

- Ganho P vertical definido via parâmetro MPC_Z_P.

# Combinando Posicao e Velocidade 
![image](https://github.com/Ararabots-UFMS/Drone/assets/104502725/3832436d-d20b-4d8c-85b4-8173d37cf981)

Alimentações antecipadas (ff) dependentes do modo - por exemplo, o gerador de trajetória do modo de missão (trajetória limitada pela aceleração) calcula pontos de referência de posição, velocidade e aceleração.

- Os pontos de referência de aceleração (referência inercial) serão transformados (com ponto de referência de guinada) em pontos de referência de atitude (quaternion) e ponto de referência de empuxo coletivo.

