# Desafios Técnicos:

1. O sistema de posicionamento calcula os dados de odometria com base nas poses do scanner a laser. Isso pode resultar em uma estimativa incorreta da posição do drone em relação ao ambiente circundante.
2. As leituras do LiDAR podem ser infinitas se a distância entre o LiDAR e as paredes circundantes exceder o alcance do LiDAR.
3. A comunicação entre o Raspberry Pi e o PC depende fortemente do Wi-Fi. Portanto, qualquer perda no sinal Wi-Fi terminaria a comunicação entre o drone e o PC.

# Desafios Não Técnicos:

1. O ambiente interno pode estar cheio de obstáculos, o que dificulta o processo de planejamento de trajetória.
2. Espelhos, janelas e portas de vidro podem afetar a precisão do mapa, já que não são detectados corretamente pelos pulsos de laser.

# Hardware a Ser Utilizado:

Seleção de Hardware:
1. Kit de Drone COEX Clover
   - Kit de drone STEM educacional completo e programável.
   - Pode operar de forma estável sem GPS.
   - Plataforma Clover utiliza o framework ROS.
   - Especialmente feito para voos internos.

2. Slamtec RPLiDAR A1M8
   - Sensor de varredura 2D de baixo custo, com 360° de alcance e 12m.
   - Scanner de alcance a laser omnidirecional 360°.
   - Compatível com ROS.
   - Taxa de amostragem muito alta de 8 mil vezes, considerada uma das mais altas na indústria de LiDAR atual.
   - Ideal para navegação interna e localização usando UAVs.

3. Raspberry Pi 4 Model B
   - Computador de placa única usado como computador companheiro.
   - Baixo consumo de energia.
   - Conecta o drone via Wi-Fi.
   - Responsável pela autonomia do voo.
   - Acessa e emite comandos para periféricos.

4. Sensor de Alcance VL53L1X
   - Módulo de sensor de alcance a laser.
   - Um dos menores sensores de laser de tempo de voo de 940 nm.
   - Mede alcance absoluto de até 4 metros.
   - Suporta alcance de 400cm, adequado para várias aplicações.

# Seleção de Software:
1. Robot Operating System (ROS)
   - Framework que roda no sistema operacional Linux e será usado como firmware para controlar e monitorar o sistema.

2. Máquina Virtual COEX
   - Sistema operacional Linux com ROS pré-instalado, juntamente com algumas dependências e pacotes necessários, além de um ambiente Gazebo pré-configurado.

3. Gazebo
   - Ferramenta de simulação que será usada para testar e experimentar diferentes abordagens de mapeamento e automação.

4. Visual Studio Code
   - Editor de texto para escrever scripts Python para programar o drone.

5. RVIZ
   - Ferramenta de visualização para visualizar as leituras do LiDAR.

6. QGroundControl
   - Suporta controle total de voo e planejamento de missões para qualquer drone habilitado para MAVLink.
  
# Implementação

A implementação foi dividida em duas partes. A primeira parte é trabalhar no software de simulação, e a segunda parte é trabalhar nos componentes de hardware físico. O software de simulação nos ajudou a ter uma estimativa de como o sistema funcionará no mundo físico. Através do software de simulação, pudemos identificar alguns desafios de implementação e encontrar soluções para eles. Além disso, o software de simulação nos deu a oportunidade de processar os dados dos sensores e testá-los antes de testá-los fisicamente, o que acelerou o processo de trabalhar nos componentes físicos e testá-los. Além disso, descobriu-se que os resultados obtidos dos componentes simulados e dos componentes físicos eram próximos entre si. Nesta seção, demonstramos o progresso que aconteceu tanto no mundo físico quanto no virtual.

# O Software de Simulação

## O Algoritmo de Mapeamento

O algoritmo Hector SLAM foi selecionado neste projeto devido à sua alta eficiência no mapeamento de ambientes internos, sua capacidade de trabalhar eficientemente com drones e sua facilidade de integração com o sensor LiDAR selecionado. Além disso, ele consome menos energia ao lidar com alguns casos em que o ambiente interno é dinâmico e os obstáculos estão em movimento. O Hector SLAM é um algoritmo amplamente utilizado no mapeamento de ambientes internos desconhecidos. O algoritmo é baseado em LiDAR e usa a equação de Gauss-Newton para construir mapas precisos a partir dos dados do scanner a laser. Além disso, esse algoritmo não usa nenhum dado de odometria para estimar a posição do robô em relação ao seu entorno. Em vez disso, o algoritmo utiliza a diferença nas localizações do scanner a laser para calcular a odometria. Essa característica qualifica o algoritmo Hector SLAM para funcionar de forma ótima com veículos aéreos não tripulados, dado que, na maioria dos casos, os dados de odometria são calculados a partir do processamento do movimento das rodas, e esse não é o caso com os UAVs. Além disso, o algoritmo fornece uma estimativa precisa da posição do robô em relação ao seu entorno.

## O Algoritmo de Exploração

O método usado neste projeto para explorar os locais internos é o algoritmo de seguimento de parede devido à sua eficácia e simplicidade. A implementação desse algoritmo pode ser resumida em três funções principais: `left_side()`, `move_forward()` e `take_stop_action()`, que são representadas em um loop `while`.

    while(1):
        left_side()
        move_forward()
        take_stop_action()

    left_side()


Esta função foi implementada para mover o drone para frente com segurança, sem bater em uma parede ou sem pular um canto externo. A lógica por trás desse algoritmo é que ele utiliza leituras do LiDAR que apontam exatamente para o oeste do drone para medir a distância do drone à parede esquerda e, em seguida, ajusta o drone para que esteja aproximadamente a 0,7 metros da parede esquerda. A razão para usar 0,7 metros é porque o drone tem um erro maior do que o esperado. Portanto, um loop while é usado para garantir que o drone esteja suficientemente longe da parede.

    move_forward():

Essa função foi implementada para mover o drone para frente com segurança, sem bater em uma parede ou sem pular um canto externo. A lógica por trás desse algoritmo é que ele utiliza o conceito de ângulo reto e várias leituras que correspondem a diferentes ângulos para medir as distâncias seguras. O fluxograma abaixo demonstra a lógica desse algoritmo em detalhes.

![image](https://github.com/Ararabots-UFMS/Drone/assets/104502725/a487b553-61f4-4ede-8a7c-260cf0c7ad85)

    take_stop_action():

Esta função lida com duas situações diferentes:

1. Na primeira situação, o drone pode parar quando enfrenta um canto interno. Isso pode ser detectado medindo a distância da parede frontal e comparando a distância atual da leitura do LiDAR esquerdo com a anteriormente registrada. Se a comparação mostrar uma pequena diferença entre essas duas leituras, isso significa que o drone deve girar para a direita e continuar seu caminho.

2. A segunda situação é que o drone pode parar quando detecta um canto externo. A lógica é exatamente como na primeira situação, exceto que o drone deve estar afastado da parede frontal (com uma distância maior que 1,5 metro). Além disso, a diferença entre a leitura atual do LiDAR esquerdo e a leitura anteriormente registrada deve ser maior que 0,5 metros. Se este for o caso, então o drone parou por causa de um canto externo. Portanto, o drone deve girar para a esquerda e continuar seu caminho.
