#!/usr/bin/env python3
from proyecto_interfaces.srv import StartManipulationTest
import rclpy
from rclpy.node import Node

from math import cos, sin, pi
from scipy.optimize import fsolve
import math
from proyecto_interfaces.msg import String


import time

Plataforma = ""
ObjetivoX = 0.0
ObjetivoY = 0.1
ObjetivoZ = 0.0

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal')

        self.srv = self.create_service(StartManipulationTest, '/group_5/start_manipulation_test_srv', self.start_manipulation_test_callback)
        timer_period = 0.5  # seconds

        self.arduino = self.create_publisher(String, 'arduinoSerial', 10)
        
    def start_manipulation_test_callback(self, request, response):
        global Plataforma,ObjetivoZ,ObjetivoX,ObjetivoY
        Plataforma = request.platform
        otra = "platform"
        if(Plataforma == "platform_1"):
            ObjetivoY = 0.1
            otra = "platform_2"
        elif(Plataforma == "platform_2"):
            otra = "platform_1"
            ObjetivoY = 3.0
        ObjetivoX = 15.0
        self.Cinematica_inversa()
        response.answer = "La ficha de tipo " + str(request.x) + " se encuentra en la plataforma "+ Plataforma +  " y la llevaré a la plataforma "+ otra
        return response

    def Cinematica_inversa(self):
        global ObjetivoY, ObjetivoX, ObjetivoZ
        msg_1 = String()
        msg_2 = String()
        msg_3 = String()
         # Resolver el sistema de ecuaciones
        initial_guess = [0, 0]  # Aproximación inicial de las variables x e y
        solution = fsolve(self.equations, initial_guess)

        # Imprimir las soluciones
        x_solution, y_solution = solution
        deg_q1 = round(90+math.atan(ObjetivoZ/ObjetivoY)*(180/math.pi))
        deg_q2 = round(180-x_solution*(180/math.pi))
        deg_q3 = round(90-y_solution*(180/math.pi))
        #print("Solución para x:", 180-x_solution*(180/math.pi))
        #print("Solución para y:", 90-y_solution*(180/math.pi))
        msg_1.data = self.dar_formato(str(deg_q1), str(deg_q2), str(deg_q3), str(150))
        self.arduino.publish(msg_1)
        time.sleep(5)

        ObjetivoX = 5
        ObjetivoY = 10.0

          # Resolver el sistema de ecuaciones
        initial_guess = [0, 0]  # Aproximación inicial de las variables x e y
        solution = fsolve(self.equations, initial_guess)

        # Imprimir las soluciones
        x_solution, y_solution = solution
        deg_q1 = round(90+math.atan(ObjetivoZ/ObjetivoY)*(180/math.pi))
        deg_q2 = round(180-x_solution*(180/math.pi))
        deg_q3 = round(90-y_solution*(180/math.pi))
        #print("Solución para x:", 180-x_solution*(180/math.pi))
        #print("Solución para y:", 90-y_solution*(180/math.pi))
        msg_2.data = self.dar_formato(str(deg_q1), str(deg_q2), str(deg_q3), str(0))
        self.arduino.publish(msg_2)

        time.sleep(30)
        ObjetivoX = 15
        if(Plataforma == "platform_1"):
            ObjetivoY = 3.0
            otra = "platform_2"
        elif(Plataforma == "platform_2"):
            otra = "platform_1"
            ObjetivoY = 0.1

        # Resolver el sistema de ecuaciones
        initial_guess = [0, 0]  # Aproximación inicial de las variables x e y
        solution = fsolve(self.equations, initial_guess)

        # Imprimir las soluciones
        x_solution, y_solution = solution
        deg_q1 = round(90+math.atan(ObjetivoZ/ObjetivoY)*(180/math.pi))
        deg_q2 = round(180-x_solution*(180/math.pi))
        deg_q3 = round(90-y_solution*(180/math.pi))
        #print("Solución para x:", 180-x_solution*(180/math.pi))
        #print("Solución para y:", 90-y_solution*(180/math.pi))
        msg_3.data = self.dar_formato(str(deg_q1), str(deg_q2), str(deg_q3), str(180))
        self.arduino.publish(msg_3)

    def dar_formato(self, Q1, Q3, Q4, Q5):
        if(len(Q1)==1):
            Q1="00"+Q1;
        if(len(Q3)==1):
            Q3="00"+Q3;
        if(len(Q4)==1):
            Q4="00"+Q4;
        if(len(Q5)==1):
            Q5="00"+Q5;

        if(len(Q1)==2):
            Q1="0"+Q1;
        if(len(Q3)==2):
            Q3="0"+Q3;
        if(len(Q4)==2):
            Q4="0"+Q4;
        if(len(Q5)==2):
            Q5="0"+Q5;
        return "A"+Q1+"B"+Q3+"C"+Q4+"D090"+"E"+Q5

    def equations(self,vars):
        global ObjetivoX, ObjetivoY
        x, y = vars
        ObjetivoX = ObjetivoX -4
        ObjetivoX = ObjetivoY +5
        if not (0 <= x <= math.pi and -math.pi/2 <= y <= math.pi/2):
            return [1e10, 1e10]  # Devolver valores grandes para salir de la optimización
        eq1 = 7.5 * cos(x) + 8 * cos(x + y) + 12 * cos(x + y - pi/2) - (ObjetivoX)
        eq2 = 7.5 * sin(x) + 8 * sin(x + y) + 12 * sin(x + y - pi/2) - (ObjetivoY)+ 13.5
        return [eq1, eq2]

def main(args=None):
 
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()