#!/usr/bin/env python3
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
import cv2
import pytesseract
from proyecto_interfaces.srv import StartPerceptionTest
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image,CompressedImage # Image is the message type
from proyecto_interfaces.msg import Banner
import numpy as np

bridge = CvBridge()
image_path = "./src/proyecto_interfaces/resources/camera_image.jpeg"

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
      
        self.start_perception_test = self.create_service(StartPerceptionTest, '/group_5/start_perception_test_srv', self.start_perception_test_callback)
        self.image_topic = self.create_subscription(
            CompressedImage,
            'video_frames',
            self.image_topic_callback,
            1)
        self.image_topic  # prevent unused variable warning

    def image_topic_callback(self, msg):
        compressed_data = msg.data
        print("Received an image!")
        with open(image_path, 'wb') as file:
            file.write(compressed_data)
        imagen = cv2.imread(image_path)
        alto, ancho = imagen.shape[:2]
        # Definir el ángulo de rotación
        angulo = 270
        # Calcular el nuevo tamaño de la imagen después de la rotación
        nuevo_ancho = int(ancho * abs(np.cos(np.radians(angulo))) + alto * abs(np.sin(np.radians(angulo))))
        nuevo_alto = int(alto * abs(np.cos(np.radians(angulo))) + ancho * abs(np.sin(np.radians(angulo))))
        # Obtener la matriz de rotación utilizando la función getRotationMatrix2D
        matriz_rotacion = cv2.getRotationMatrix2D((ancho / 2, alto / 2), angulo, 1)
        # Ajustar la matriz de rotación para evitar bordes negros
        matriz_rotacion[0, 2] += (nuevo_ancho - ancho) / 2
        matriz_rotacion[1, 2] += (nuevo_alto - alto) / 2
        # Aplicar la rotación a la imagen utilizando la función warpAffin
        imagen_rotada = cv2.warpAffine(imagen, matriz_rotacion, (nuevo_ancho, nuevo_alto))

        resize = cv2.resize(imagen_rotada, (54, 140))
        #ancho = cv2_img.shape[1] #columnas
        #alto = cv2_img.shape[0] # filas
        # Rotación
        #M = cv2.getRotationMatrix2D((ancho//2,alto//2),90,1)
        #imageOut = cv2.warpAffine(cv2_img,M,(ancho,alto))
        cv2.imwrite(image_path, imagen_rotada)

    def start_perception_test_callback(self, request, response):
        # Receive the request data and sum it
        response.answer = "Debo identificar el banner a que se encuentra en las coordenadas x_1, y_1 y el banner b que se encuentra en las coordenadas x_2, y_2"
        # Return the sum as the reply
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.banner_a, request.banner_b))
        self.perception_method()
        return response

    def get_color(hue_value):
        colors = {
            (0, 5): "Rojo",
            (5, 20): "Naranja",
            (20, 33): "Amarillo",
            (33, 78): "Verde",
            (78, 131): "Azul",
            (131, 170): "Morado"
        }
        for key, value in colors.items():
            if key[0] <= hue_value < key[1]:
                return value
        return "Indefinido"

    def get_figure(sides):
        figures = {
            3: "Triángulo",
            4: "Cuadrado o Rectángulo",
            5: "Pentágono",
            6: "Hexágono"
        }
        return figures.get(sides, "Círculo")
    # define a video capture object
        vid = cv2.VideoCapture(0)
        while True:

            # Capture the video frame
            # by frame
            ret, frame = vid.read()
            #Convertir a escala de grises
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Convertir a espacio de color HSV
            hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Obtener dimensiones del frame
            height, width, _ = frame.shape
            cx = int(50 * width / 100)
            cy = int(80 * height / 100)
            pixel_center = hsv_image[cy, cx]
            hue_value = pixel_center[0]

            color = get_color(hue_value)

            # Detectar contornos
            edges = cv2.Canny(gray, 200, 750)
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            figura = "Indefinido"

            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 3500:
                    continue

            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
            sides = len(approx)
            cv2.drawContours(frame, [approx], 0, (0, 255, 0), 2)
            x = approx.ravel()[0]
            y = approx.ravel()[1] - 10
            figure_name = get_figure(sides)
            cv2.putText(frame, figure_name, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            figura = figure_name
            #Aplicar OCR al frame
            texto = pytesseract.image_to_string(gray)
            #print("Texto extraído:", texto)

            #Mostrar resultados en pantalla
            cv2.putText(frame, "Figura: " + figura, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, "Color: " + color, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(frame, "Texto: " + texto, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Display the resulting frame
            cv2.imshow('frame', frame)

            # the 'q' button is set as the quitting button you may use any
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        # After the loop release the cap object
        vid.release()
        # Destroy all the windows
        cv2.destroyAllWindows()
        print("Figura" + figura)
        print("Color" + color)
        print("Texto:", texto)
        res=Banner()
        res.banner=1
        res.figure=figura
        res.word=color
        res.color=texto
        self.vision.publish(res)

def main(args=None):
 
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()