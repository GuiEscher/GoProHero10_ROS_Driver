GoProHero10_ROS - Preparando a webcam
README - GoProHero10_ROS
# GoProHero10_ROS - Preparando a webcam
Clone o repositrio:
```bash
git clone https://github.com/jschmid1/gopro_as_webcam_on_linux
cd gopro_as_webcam_on_linux
sudo ./install.sh
```
O script ser instalado em /usr/local/sbin/gopro
## Dependncias
```bash
sudo apt install ffmpeg v4l2loopback-dkms curl vlc
```
Se necessrio, instale manualmente o v4l2loopback-dkms:
https://github.com/umlaeute/v4l2loopback
Libere a porta 8554/UDP no firewall:
```bash
sudo firewall-cmd --add-port=8554/udp
sudo firewall-cmd --add-port=8554/udp --permanent
```
## Modo de Uso
```bash
sudo gopro webcam
sudo gopro webcam -a -n
sudo gopro webcam -p enx
sudo gopro webcam -d <dispositivo>
sudo gopro webcam -r 1080 | 720 | 480
sudo gopro webcam -f wide | linear | narrow
sudo gopro webcam -v
sudo gopro webcam -V
sudo gopro webcam -u <seu-usuario>
```
## Inicializao Automtica no Boot
```bash
sudo nano gopro_webcam.service
```
Cole o seguinte contedo:
```
[Unit]
Description=GoPro Webcam start script
After=network-online.target
Wants=network-online.target systemd-networkd-wait-online.service
[Service]
GoProHero10_ROS - Preparando a webcam
ExecStart=/usr/local/sbin/gopro webcam -a -n -r 1080
Restart=on-failure
RestartSec=15s
[Install]
WantedBy=multi-user.target
```
Salve e copie:
```bash
sudo cp gopro_webcam.service /etc/systemd/system/
sudo systemctl start gopro_webcam.service
sudo systemctl enable gopro_webcam.service
```
## Driver ROS - GoProHero10_ROS_Driver
Verifique o ndice da cmera:
```bash
v4l2-ctl --list-devices
ls /dev/video*
```
Edite o config.yml:
```yaml
camera_index: 1
frame_rate: 15
resolution_width: 800
resolution_height: 600
photos_dir: "~/Pictures/camera_photos/"
videos_dir: "~/Videos/camera_videos/"
record_duration: 10
```
Compile o workspace ROS:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
Execute o driver:
```bash
rosrun camera_driver camera_driver.py
```
Controles via terminal:
```
p Tirar uma foto
r Iniciar/parar gravao de vdeo
q Encerrar o programa
```
GoProHero10_ROS - Preparando a webcam
Instale o cv_bridge (caso necessrio):
```bash
sudo apt install ros-noetic-cv-bridge
```
Ou o metapacote:
```bash
sudo apt install ros-noetic-vision-opencv
```
