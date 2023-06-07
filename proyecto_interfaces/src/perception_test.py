#!/usr/bin/env python3
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
import cv2
import pytesseract
from proyecto_interfaces.srv import StartPerceptionTest
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image # Image is the message type
from proyecto_interfaces.msg import Banner
from sensor_msgs.msg import CompressedImage

bridge = CvBridge()
image_path = "./src/proyecto_interfaces/resources/camera_image.jpeg"

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
      
        self.start_perception_test = self.create_service(StartPerceptionTest, '/group_5/start_perception_test_srv', self.start_perception_test_callback)
        self.image_topic = self.create_subscription(
            Image,
            'video_frames',
            self.image_topic_callback,
            10)
        self.image_topic  # prevent unused variable warning

        self.vision = self.create_publisher(Banner, '/vision/banner_group_5', 10)

    def image_topic_callback(self, msg):
        print("Received an image!")
        cv2_img = bridge.imgmsg_to_cv2(msg, "rgb8")
        #M = cv2.getRotationMatrix2D((cv2_img.shape[1],cv2_img.shape[0]),90,1)
        #imageOut = cv2.warpAffine(cv2_img,M,(cv2_img.shape[1],cv2_img.shape[0]))
        cv2.imwrite(image_path, cv2_img)

    def start_perception_test_callback(self, request, response):
        # Receive the request data and sum it
        response.answer = "Debo identificar el banner a que se encuentra en las coordenadas x_1, y_1 y el banner b que se encuentra en las coordenadas x_2, y_2"
        # Return the sum as the reply
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.banner_a, request.banner_b))
        self.perception_method()
        return response

    def perception_method(self):
        image = cv2.imread(image_path)

        # Convertir la imagen a escala de grises
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Convertir la imagen a espacio de color HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Obtener las dimensiones de la imagen
        height, width, _ = image.shape

        # Calcular las coordenadas del centro de la imagen
        cx = int(50 * width / 100)
        cy = int(80 * height / 100)
        # Obtener el valor de tonalidad (hue) del píxel central
        pixel_center = hsv_image[cy, cx]
        hue_value = pixel_center[0]

        # Determinar el color basado en el valor de tonalidad
        color = "Indefinido"
        if hue_value < 5:
            color = "Rojo"
        elif hue_value < 20:
            color = "Naranja"
        elif hue_value < 33:
            color = "Amarillo"
        elif hue_value < 78:
            color = "Verde"
        elif hue_value < 131:
            color = "Azul"
        elif hue_value < 170:
            color = "Morado"

        # Mostrar el valor del píxel central
        print("Valor del píxel central:", pixel_center)

        # Convertir la imagen a escala de grises
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        texto = pytesseract.image_to_string(gray)
        print("texo " + texto)
        # Aplicar detección de bordes utilizando Canny
        edges = cv2.Canny(gray, 200, 750)

        # Encontrar contornos en los bordes
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print(contours)

        # Iterar sobre los contornos encontrados
        figura="Indefinido"
        for contour in contours:
            area = cv2.contourArea(contour)

            # Filtrar contornos con un área menor que el umbral deseado
            if area < 3000:
                continue

            # Aproximar el contorno a una forma más simple
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)

            # Obtener el número de lados de la figura
            sides = len(approx)

            # Dibujar un contorno alrededor de la figura identificada
            cv2.drawContours(image, [approx], 0, (0, 255, 0), 2)

            # Mostrar el número de lados cerca de la figura identificada
            x = approx.ravel()[0]
            y = approx.ravel()[1] - 10
            if sides == 3:
                cv2.putText(image,"Triangle", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                figura="Triangulo"
            elif sides == 4:
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = float(w) / h
                if aspect_ratio >= 0.95 and aspect_ratio <= 1.05:
                    cv2.putText(image, "Square", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    figura="Cuadrado"
                else:
                    cv2.putText(image, "Rectangle", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    figura="Rectangulo"
            elif sides == 5:
                cv2.putText(image, "Pentagon", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                figura="Pentagono"
            elif sides == 6:
                cv2.putText(image, "Hexagon", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                figura="Hexagono"
            else:
                cv2.putText(image, "Circle", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                figura="Circulo"

        # Dibujar el nombre del color en la imagen
        cv2.putText(image, color, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Mostrar la imagen resultante con figuras, texto y color
        cv2.imshow("Image", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        print("Figura" + figura)
        print("Color" + color)
        print("Texto:", texto)
        res=Banner()
        res.banner=1
        res.figure=figura
        res.word=color
        res.color=texto
        compressed_img = bridge.cv2_to_compressed_imgmsg(image)
        self.vision.publish(compressed_img)

def main(args=None):
 
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
