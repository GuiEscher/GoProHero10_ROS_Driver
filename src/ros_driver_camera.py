#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
from datetime import datetime
import time
import yaml
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, 
                             QVBoxLayout, QWidget, QLabel, QSpinBox,
                             QFileDialog, QHBoxLayout, QMessageBox)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap
import sys

class CameraDriverGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Configurações básicas da janela
        self.setWindowTitle("GoPro ROS Driver")
        self.setGeometry(100, 100, 800, 600)  # Aumentei o tamanho para melhor visualização
        
        # Inicializa o driver ROS
        self.init_ros_driver()
        
        # Cria a interface
        self.init_ui()
        
        # Timer para atualizar a visualização
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(1000 // self.frame_rate)
        
    def init_ros_driver(self):
        rospy.init_node('camera_driver_gui', anonymous=True)
        
        # Carregando configurações a partir de um arquivo YAML
        config_path = rospy.get_param('~config_file', '/home/guilherme/catkin_ws/src/camera_driver/src/config.yml')
        self.load_config(config_path)

        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            rospy.logerr(f"Não foi possível acessar a câmera no índice {self.camera_index}")
            QMessageBox.critical(self, "Erro", "Não foi possível acessar a câmera.")
            sys.exit(1)

        # Configura as propriedades da câmera após verificar que está aberta
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        # Verifica as configurações reais obtidas
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        rospy.loginfo(f"Configurações da câmera: {actual_width}x{actual_height} @ {actual_fps}fps")

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("camera/image", Image, queue_size=10)

        os.makedirs(self.photos_dir, exist_ok=True)
        os.makedirs(self.videos_dir, exist_ok=True)

        self.is_recording = False
        self.video_writer = None
        self.record_start_time = 0
        
    def load_config(self, config_path):
        with open(config_path, 'r') as config_file:
            config = yaml.safe_load(config_file)
            self.camera_index = config.get('camera_index', 0)
            self.frame_rate = config.get('frame_rate', 15)
            self.resolution_width = config.get('resolution_width', 800)
            self.resolution_height = config.get('resolution_height', 600)
            self.photos_dir = os.path.expanduser(config.get('photos_dir', '~/Pictures/camera_photos/'))
            self.videos_dir = os.path.expanduser(config.get('videos_dir', '~/Videos/camera_videos/'))
            self.record_duration = config.get('record_duration', 10)
    
    def init_ui(self):
        # Widget central
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Layout principal
        layout = QVBoxLayout()
        
        # Visualização da câmera (agora maior)
        self.camera_view = QLabel()
        self.camera_view.setAlignment(Qt.AlignCenter)
        self.camera_view.setStyleSheet("background-color: black;")
        self.camera_view.setMinimumSize(640, 480)
        layout.addWidget(self.camera_view)
        
        # Controles de configuração
        config_layout = QHBoxLayout()
        
        self.duration_spin = QSpinBox()
        self.duration_spin.setRange(1, 3600)  # 1 segundo a 1 hora
        self.duration_spin.setValue(self.record_duration)
        self.duration_spin.setSuffix(" segundos")
        
        config_layout.addWidget(QLabel("Duração da gravação:"))
        config_layout.addWidget(self.duration_spin)
        layout.addLayout(config_layout)
        
        # Botões de controle
        btn_layout = QHBoxLayout()
        
        self.photo_btn = QPushButton("Tirar Foto")
        self.photo_btn.clicked.connect(self.capture_photo)
        
        self.record_btn = QPushButton("Iniciar Gravação")
        self.record_btn.clicked.connect(self.toggle_recording)
        self.record_btn.setStyleSheet("background-color: red; color: white;")
        
        btn_layout.addWidget(self.photo_btn)
        btn_layout.addWidget(self.record_btn)
        layout.addLayout(btn_layout)
        
        # Botão para alterar diretórios
        dir_layout = QHBoxLayout()
        
        self.photos_dir_btn = QPushButton("Alterar Pasta de Fotos")
        self.photos_dir_btn.clicked.connect(self.change_photos_dir)
        
        self.videos_dir_btn = QPushButton("Alterar Pasta de Vídeos")
        self.videos_dir_btn.clicked.connect(self.change_videos_dir)
        
        dir_layout.addWidget(self.photos_dir_btn)
        dir_layout.addWidget(self.videos_dir_btn)
        layout.addLayout(dir_layout)
        
        central_widget.setLayout(layout)
        
        # Status bar
        self.statusBar().showMessage(f"Camera {self.camera_index} - {self.resolution_width}x{self.resolution_height} @ {self.frame_rate}fps")
    
    def update_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # Redimensiona o frame para a visualização
            frame_resized = cv2.resize(frame, (640, 480))
            
            # Converte para formato Qt
            rgb_image = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            
            # Atualiza a visualização
            self.camera_view.setPixmap(QPixmap.fromImage(qt_image))
            
            # Publica no tópico ROS
            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(ros_image)
            except CvBridgeError as e:
                rospy.logerr(f"Erro ao converter imagem: {e}")
            
            # Se estiver gravando, escreve no vídeo
            if self.is_recording:
                if self.video_writer is not None:
                    self.video_writer.write(frame)
                
                # Verifica se atingiu o tempo de gravação
                elapsed = time.time() - self.record_start_time
                if elapsed >= self.record_duration:
                    self.stop_recording()
    
    def capture_photo(self):
        ret, frame = self.cap.read()
        if ret:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            photo_path = os.path.join(self.photos_dir, f"photo_{timestamp}.jpg")
            cv2.imwrite(photo_path, frame)
            
            QMessageBox.information(self, "Foto Capturada", f"Foto salva em:\n{photo_path}")
    
    def toggle_recording(self):
        if self.is_recording:
            self.stop_recording()
        else:
            self.start_recording()
    
    def start_recording(self):
        self.record_duration = self.duration_spin.value()
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.video_path = os.path.join(self.videos_dir, f"video_{timestamp}.avi")
        
        # Obtém as propriedades reais do frame
        ret, frame = self.cap.read()
        if not ret:
            QMessageBox.critical(self, "Erro", "Não foi possível capturar frame para verificação")
            return
            
        frame_height, frame_width = frame.shape[:2]
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        # Usa o codec MJPG que geralmente funciona bem
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.video_writer = cv2.VideoWriter(
            self.video_path,
            fourcc,
            actual_fps,
            (frame_width, frame_height))
        
        if not self.video_writer.isOpened():
            QMessageBox.critical(self, "Erro", f"Falha ao iniciar gravação em:\n{self.video_path}")
            return
        
        self.is_recording = True
        self.record_start_time = time.time()
        self.record_btn.setText("Parar Gravação")
        self.record_btn.setStyleSheet("background-color: green; color: white;")
        
        self.statusBar().showMessage(f"Gravando... {self.video_path}")
    
    def stop_recording(self):
        if self.is_recording and self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
            self.is_recording = False
            self.record_btn.setText("Iniciar Gravação")
            self.record_btn.setStyleSheet("background-color: red; color: white;")
            
            # Verifica se o arquivo de vídeo foi criado
            if os.path.exists(self.video_path) and os.path.getsize(self.video_path) > 0:
                QMessageBox.information(self, "Gravação Concluída", f"Vídeo salvo em:\n{self.video_path}")
                self.statusBar().showMessage(f"Gravação concluída: {self.video_path}")
            else:
                QMessageBox.warning(self, "Aviso", "O arquivo de vídeo não foi gravado corretamente.")
                self.statusBar().showMessage("Falha na gravação do vídeo")
    
    def change_photos_dir(self):
        new_dir = QFileDialog.getExistingDirectory(self, "Selecionar Pasta para Fotos", self.photos_dir)
        if new_dir:
            self.photos_dir = new_dir
            QMessageBox.information(self, "Pasta Alterada", f"Novo diretório para fotos:\n{self.photos_dir}")
    
    def change_videos_dir(self):
        new_dir = QFileDialog.getExistingDirectory(self, "Selecionar Pasta para Vídeos", self.videos_dir)
        if new_dir:
            self.videos_dir = new_dir
            QMessageBox.information(self, "Pasta Alterada", f"Novo diretório para vídeos:\n{self.videos_dir}")
    
    def closeEvent(self, event):
        if self.is_recording:
            self.stop_recording()
        
        if self.cap.isOpened():
            self.cap.release()
        if hasattr(self, 'video_writer') and self.video_writer is not None:
            self.video_writer.release()
        cv2.destroyAllWindows()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    try:
        window = CameraDriverGUI()
        window.show()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass