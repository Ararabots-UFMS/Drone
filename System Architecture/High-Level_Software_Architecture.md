# Arquitetura High-Level Software

O diagrama abaixo fornece uma visão detalhada dos blocos de construção do PX4. A parte superior do diagrama contém blocos de middleware, enquanto a seção inferior mostra os componentes da pilha de voo.
![image](https://github.com/Ararabots-UFMS/Drone/assets/104502725/b6452803-3b0b-42f2-9153-0205a10612de)

As setas mostram o fluxo de informações para as conexões mais importantes entre os módulos. Na realidade, existem muitas mais conexões do que as mostradas, e alguns dados (por exemplo, para parâmetros) são acessados pela maioria dos módulos.

# Fight Stack


Fight Stack é uma coleção de algoritmos de orientação, navegação e controle para drones autônomos. Inclui controladores para estruturas aerodinâmicas de asa fixa, multirotores e VTOL, bem como estimadores para atitude e posição.

O diagrama a seguir mostra uma visão geral dos blocos de construção da pilha de voo. Ele contém o pipeline completo desde os sensores, entrada RC e controle de voo autônomo (Navegador), até o controle de motor ou servo (Atuadores).
![image](https://github.com/Ararabots-UFMS/Drone/assets/104502725/9cb63401-3301-42eb-b06d-685c48c62cc0)

Um estimador recebe uma ou mais entradas de sensor, as combina e calcula o estado do veículo (por exemplo, a atitude a partir dos dados do sensor IMU).

Um controlador é um componente que recebe um ponto de referência e uma medição ou estado estimado (variável de processo) como entrada. Seu objetivo é ajustar o valor da variável de processo de forma que corresponda ao ponto de referência. A saída é uma correção para eventualmente alcançar esse ponto de referência. 
Por exemplo, o controlador de posição recebe pontos de referência de posição como entradas, a variável de processo é a posição atualmente estimada, e a saída é um ponto de referência de atitude e empuxo que movem o veículo em direção à posição desejada.

Um misturador (mixer) recebe comandos de força (como "virar à direita") e os traduz em comandos de motor individuais, garantindo que alguns limites não sejam excedidos. Essa tradução é específica para um tipo de veículo e depende de vários fatores, como os arranjos dos motores em relação ao centro de gravidade ou a inércia rotacional do veículo.

# Middleware

O middleware consiste principalmente de drivers de dispositivos para sensores embarcados, comunicação com o mundo externo (computador companheiro, GCS, etc.) e o barramento de mensagens publicar-assinar uORB.

Além disso, o middleware inclui uma camada de simulação que permite que o código de voo PX4 seja executado em um sistema operacional de desktop e controle um veículo modelado por computador em um "mundo" simulado.
