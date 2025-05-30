

# GoProHero10_ROS - Preparando a webcam

incialmente é necessário clonar o repositório disponível em: https://github.com/jschmid1/gopro_as_webcam_on_linux e seguir os passos

1. Conexão da Câmera
Inicialmente, conecte a GoPro Hero 10 ao computador via cabo USB. Para melhor estabilidade, recomenda-se remover a bateria e deixar a câmera alimentada apenas pela conexão USB.

2. Instalação do Driver
Clone o repositório gopro_as_webcam_on_linux:

> git clone https://github.com/jschmid1/gopro_as_webcam_on_linux
-> cd gopro_as_webcam_on_linux
-> sudo ./install.sh

O script será instalado em /usr/local/sbin/gopro

3. Dependências Necessárias
Antes de usar, instale os seguintes pacotes:

-> sudo apt install ffmpeg v4l2loopback-dkms curl vlc

Se sua distribuição não tiver o v4l2loopback-dkms, instale manualmente pelo link:
https://github.com/umlaeute/v4l2loopback

Além disso, libere a porta 8554/UDP no firewall:

-> sudo firewall-cmd --add-port=8554/udp
-> sudo firewall-cmd --add-port=8554/udp --permanent

4. Modo de Uso
Para iniciar o modo webcam de forma interativa:

-> sudo gopro webcam

Para um modo automatizado:

-> sudo gopro webcam -a -n

Outras opções úteis:

-p enx: define um padrão de interface (útil se a detecção falhar)

-d <dispositivo>: define a interface de rede exata (muda a cada boot)

-r 1080 | 720 | 480: define a resolução

-f wide | linear | narrow: define o campo de visão

-v: modo verboso (mostra comandos executados)

-V: preview via VLC (não expõe o vídeo ao sistema)

-u <seu-usuário>: define o usuário (necessário para VLC)


5. Inicialização Automática no Boot
Para que o modo webcam seja iniciado automaticamente com o sistema:

crie o arquivo gopro_webcam.service:

-> sudo nano gopro_webcam.service

cole o seguinte conteúdo:

--

[Unit]
Description=GoPro Webcam start script
After=network-online.target
Wants=network-online.target systemd-networkd-wait-online.service

[Service]
ExecStart=/usr/local/sbin/gopro webcam -a -n -r 1080
Restart=on-failure
RestartSec=15s

[Install]
WantedBy=multi-user.target

--


Salve e Copie o serviço para a pasta correta:

-> sudo cp gopro_webcam.service /etc/systemd/system/

inicie e ative o serviço:

-> sudo systemctl start gopro_webcam.service
-> sudo systemctl enable gopro_webcam.service

Pode ser necessário instalar:

-> sudo apt-get install firewalld

# Preparando o driver - GoProHero10_ROS_Driver

Agora que foi configurada a gopro como webcam no linux, basta configurar o driver atual.

Para usar o Camera_driver e controlar sua GoPro via terminal para tirar fotos e gravar vídeos, é necessário primeiro configurar a GoPro Hero 10 como uma webcam utilizando o driver gopro_as_webcam_on_linux. Após seguir esse procedimento, o dispositivo da GoPro será reconhecido pelo sistema como uma webcam e poderá ser acessado pelo índice correspondente (geralmente /dev/video0, /dev/video1 etc.).

** Etapas para usar o Camera_driver: **

1. Configure sua GoPro como webcam, seguindo o tutorial do driver gopro_as_webcam_on_linux.

2. Verifique qual índice de câmera sua GoPro recebeu após a conexão:

-> v4l2-ctl --list-devices

Ou use o comando:

-> ls /dev/video*

Teste com o cheese, vlc, ou ffplay para confirmar o índice correto da câmera.

3. Edite o arquivo de configuração config.yml do Camera_driver: Altere o valor de camera_index para o índice da GoPro detectado (por exemplo, 0, 1, 2 etc.). Exemplo:

camera_index: 1
frame_rate: 15
resolution_width: 800
resolution_height: 600
photos_dir: "~/Pictures/camera_photos/"
videos_dir: "~/Videos/camera_videos/"
record_duration: 10

4. Compile seu workspace ROS, se necessário:

-> cd ~/catkin_ws
-> catkin_make
-> source devel/setup.bash

5. Execute o driver (não esqueça de iniciar o nó master usando roscore):

-> rosrun camera_driver camera_driver.py

6. Controles disponíveis via terminal:
Durante a execução do Camera_driver.py, você pode usar as seguintes teclas no terminal:

p — Tirar uma foto (salva em photos_dir)

r — Iniciar/parar gravação de vídeo (máximo de 20 segundos)

q — Encerrar o programa

As fotos e vídeos serão automaticamente salvos nas pastas definidas no arquivo de configuração.

----------------------------------------------------------------------

NOTAS: O driver também publica o stream da câmera no tópico ROS /camera/image com o tipo sensor_msgs/Image. Isso permite a integração facilmente com outros nós ROS que consomem imagens.

O driver grava vídeos em .avi com codec MJPG e captura imagens em .jpg.

A gravação para automaticamente após o tempo definido no parâmetro record_duration.

O ROS deve estar configurado corretamente e o cv_bridge instalado.

Para instalar o cv_bridge no ROS Noetic:

-> sudo apt install ros-noetic-cv-bridge

Ele faz parte do metapacote vision_opencv, que também pode ser instalado assim:

-> sudo apt install ros-noetic-vision-opencv


