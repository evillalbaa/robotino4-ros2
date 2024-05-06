#!/usr/bin/env python3

# Create a path in rviz
# https://answers.ros.org/question/278616/how-to-create-a-publisher-about-trajectory-path-then-show-it-in-rviz/
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import matplotlib.pyplot as plt
import numpy as np

class robotino_kinematics(Node):

    def __init__(self):
        super().__init__("robotino_kinematics")
        self.get_logger().info("Robotino Kinematics Node Started")

        # TIEMPO
        self.simulation_time = 1.0
        self.step_time = 0.1
        self.time_vector = np.arrage(0, self.simulation_time+self.step_time, self.step_time) # star, stop, step
        self.sample = len(self.time_vector+1)

        # CONDICIONES INICIALES
        self.vx_ini = 0.0
        self.vy_ini = 0.0
        self.w_ini = 0.0

        self

        # INICIALIZACIÓN DE VARAIBLES
        self.last_published_vx = 0.0
        self.last_published_vy = 0.0
        self.last_published_w = 0.0

        self.w_history = np.zeros(self.sample)  # Historial de velocidades vx
        self.vx_history = np.zeros(self.sample) # Historial de velocidades vx
        self.vy_history = np.zeros(self.sample) # Historial de velocidades vy
    
        self.w_history[0] = self.w_ini
        self.vx_history[0] = self.vx_ini
        self.vy_history[0] = self.vy_ini

        # INICIALIZACIÓN TÓPICOS 
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # TIMER PARA PUBLICAR EN EL TOPICO /CMD_VEL
        self.timer = self.create_timer(self.step_time, self.update_velocities)

        self.get_new_velocities()

        # EMPIEZA EL TEMPORIZADOR
        self.start_time = time.time()
    
    def update_velocities(self):

        if time.time() - self.start_time <= self.simulation_time:

            # Actualizar historial de velocidades
            self.vy_history.append(self.w)
            self.vx_history.append(self.vx)
            self.vy_history.append(self.vy)

            # Solo enviar la velocidad si ha cambiado desde la última publicación
            if (self.vx != self.last_published_vx) or (self.vy != self.last_published_vy) or (self.w != self.last_published_w):
                self.publish_velocities(self.vx,self.vy,self.w)

                # Actualizar las velocidades publicadas por última vez
                self.last_published_vx = self.vx
                self.last_published_vy = self.vy
                self.last_published_w = self.w

        else:
            self.stop_robot()
            self.publish_velocities(self.vx,self.vy,self.w)

            self.plot_trayectory()

            self.get_new_velocities()
            self.start_time = time.time()

    def stop_robot(self):
        self.vx = 0.0
        self.vy = 0.0
        self.w = 0.0

    def publish_velocities(self,vx,vy,w):
        msg = Twist()

        msg.linear.x = vx
        msg.linear.x = vy
        msg.angular.z = w

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: vx={msg.linear.x}, vy={msg.linear.y}, w={msg.angular.z}')

    def get_new_velocities(self):
        while True:
            try:
                self.vx = float(input("Ingrese la velocidad lineal en x (m/s): "))
                self.vy  = float(input("Ingrese la velocidad lineal en y (m/s): "))
                self.w = float(input("Ingrese la velocidad angular (rad/s): "))
                break

            except ValueError:
                # Captura el error si no se ingresó un número válido
                print("Por favor, ingrese un número válido.")  

    def plot_vx_vs_vy(self):
        hx_list = []
        hy_list = []
        phi_list = []

        for i in range(len(self.vx_history)):
            phi = self.vx_history[i]+self.step_time*
        
        hxp = self.vx_history[i]*cos 
        hyp =     
        
        plt.figure()
        plt.plot(self.vx_history, self.vy_history)
        plt.xlabel('vx (m/s)')
        plt.ylabel('vy (m/s)')
        plt.title('vx vs vy')
        plt.grid(True)
        plt.show()
 

def main(args=None):
    rclpy.init(args=args)
    node = robotino_kinematics()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

