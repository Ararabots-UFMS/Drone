# Configuração do Ambiente para DETECÇÃO DE OBJETOS EM TEMPO REAL

Essa configuração é feita considerando o sistema operacional Ubuntu, visto que alguns do programas não possuem versão para Windows e Mac.

Para acessar a rasp conecte ela em um monitor e ligue em algum dispositivo de energia, aconselho a conectar um teclado e mouse na mesma.

  Faça o download da pasta SSDTESTE

Abra o terminal da Rasp e execute os seguintes códigos:
--> Entre na pasta onde foi feito o download da pasta zip
    ```cd /nome da pasta/```

--> Precisamo alterar uma configuração antes de realizar todos os passos
No terminal copie o seguinte comando:
```sudo nano /etc/dphys-swapfile```

  Aperte ENTER - Utilize as setas para navegar
  Na linha CONF_SWAPSIZE=100 altere para CONF_SWAPSIZE=2048

  Pressione CTRL+O para salvar a alteração feita
  Pressione ENTER
  Pressione CTRL+X para sair dessa tela

--> Continue com os seguintes comandos:

```shell sudo apt-get update && sudo apt-get upgrade```

```shell sudo apt-get install python3-pip python3-virtualenv```

--> Instalação das bibliotecas - pode demorar um pouco até a instalação de todas

``` shell sudo apt install -y build-essential cmake pkg-config libjpeg-dev libtiff5-dev libpng-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libfontconfig1-dev libcairo2-dev libgdk-pixbuf2.0-dev libpango1.0-dev libgtk2.0-dev libgtk-3-dev libatlas-base-dev gfortran libhdf5-dev libhdf5-serial-dev libhdf5-103 libqt5gui5 libqt5webkit5 libqt5test5 python3-pyqt5 python3-dev```

--> Criação do ambiente virtual

``` shell virtualenv env```

--> Para ativar o ambiente virtual execute o seguinte comando:

```shell source env/bin/activate```

  Continuando a instalação

```shell pip install opencv-python```

```shell pip install cvzone```

Pronto, instalação concluida :)

Caso o VsCode ainda não esteja instalado, basta abrir novamente o terminal e executar:

```shell sudo apt install code```

Abra a pasta de arquivos vsCode e execute a main

* Caso precise mudar a entrada da camera, só alterar de 0 para o número desejado
* Caso queira testar com os vídeos que estão na pasta basta descomentar a seguinte linha:
  # videoPath = 'example1.mp4'
E comentar a linha:
  videoPath = 0

Execute o código e pronto


