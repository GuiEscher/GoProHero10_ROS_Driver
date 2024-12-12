#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
from datetime import datetime
import tkinter as tk
from tkinter import Button, Label


class CameraDriver:
    def __init__(self):
        """
        Inicializa o driver da câmera.
        """
        # Inicializa o nó ROS
        rospy.init_node('camera_driver', anonymous=True)
        
        # Parâmetros configuráveis via ROS Param Server
        self.camera_index = rospy.get_param('~camera_index', 0)  # Índice da câmera (padrão: 0)
        self.frame_rate = rospy.get_param('~frame_rate', 30)  # Taxa de quadros (padrão: 30 FPS)
        
        # Publicador do tópico de imagem
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
        
        # Inicializa o OpenCV para capturar a câmera
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            rospy.logerr("Não foi possível acessar a câmera com índice {}".format(self.camera_index))
            rospy.signal_shutdown("Não foi possível acessar a câmera.")
        
        # Objeto para conversão de imagens OpenCV para ROS
        self.bridge = CvBridge()

        # Cria uma pasta para salvar as fotos, se não existir
        self.photos_dir = os.path.expanduser('~/Pictures/camera_photos/')
        if not os.path.exists(self.photos_dir):
            os.makedirs(self.photos_dir)
            rospy.loginfo(f"Pasta criada para salvar fotos: {self.photos_dir}")
        
        # Configura a interface gráfica
        self.setup_gui()

    def setup_gui(self):
        """
        Configura a interface gráfica (GUI) com o Tkinter.
        """
        self.root = tk.Tk()
        self.root.title("Visualização da Câmera")

        # Cria o rótulo para exibir a imagem da câmera
        self.video_label = Label(self.root)
        self.video_label.pack()

        # Botão para capturar a foto
        self.capture_button = Button(self.root, text="Capturar Foto", command=self.capture_photo)
        self.capture_button.pack()

        # Botão para encerrar o programa
        self.quit_button = Button(self.root, text="Sair", command=self.shutdown)
        self.quit_button.pack()

    def update_frame(self):
        """
        Captura a imagem da câmera, exibe na interface gráfica e publica no tópico ROS.
        """
        ret, frame = self.cap.read()
        if not ret:
            rospy.logwarn("Falha ao capturar o quadro da câmera.")
            return

        try:
            # Converte a imagem para o formato de cores RGB (Tkinter usa RGB, OpenCV usa BGR)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Converte o frame para o formato de imagem compatível com o Tkinter
            frame_tk = cv2.imencode('.ppm', frame_rgb)[1].tobytes()

            # Atualiza a imagem exibida no rótulo
            img_tk = tk.PhotoImage(data=frame_tk)
            self.video_label.imgtk = img_tk
            self.video_label.configure(image=img_tk)

            # Converte a imagem OpenCV para uma mensagem ROS
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            
            # Publica a imagem no tópico ROS
            self.image_pub.publish(image_msg)
        except CvBridgeError as e:
            rospy.logerr("Erro na conversão de imagem: {}".format(e))

        # Atualiza o frame a cada 1 ms
        self.root.after(10, self.update_frame)

    def capture_photo(self):
        """
        Salva uma imagem da câmera na pasta de fotos.
        """
        ret, frame = self.cap.read()
        if not ret:
            rospy.logwarn("Falha ao capturar o quadro da câmera.")
            return

        try:
            # Define o nome do arquivo com timestamp
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            photo_path = os.path.join(self.photos_dir, f"photo_{timestamp}.jpg")
            
            # Salva a imagem no disco
            cv2.imwrite(photo_path, frame)
            rospy.loginfo(f"Foto salva: {photo_path}")
        except Exception as e:
            rospy.logerr(f"Erro ao salvar a foto: {e}")

    def shutdown(self):
        """
        Encerra a captura e libera os recursos.
        """
        rospy.loginfo("Encerrando o driver da câmera.")
        self.cap.release()
        cv2.destroyAllWindows()
        self.root.destroy()

    def run(self):
        """
        Inicia o loop principal da interface gráfica e o ROS.
        """
        self.update_frame()
        self.root.mainloop()


if __name__ == '__main__':
    try:
        driver = CameraDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        driver.shutdown()
