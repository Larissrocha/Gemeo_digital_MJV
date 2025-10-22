#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoverCarrinho(Node):
    def __init__(self):
        super().__init__('mover_carrinho')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Publicador iniciado em /cmd_vel')
        time.sleep(2) 
        
    def mover_frente(self, velocidade=0.3, duracao=3.0):
        msg = Twist()
        msg.linear.x = velocidade
        self.publisher_.publish(msg)
        self.get_logger().info('Andando pra frente...')
        time.sleep(duracao)
        self.parar()

    def girar_esquerda(self, velocidade=0.5, duracao=2.0):
        msg = Twist()
        msg.angular.z = velocidade
        self.publisher_.publish(msg)
        self.get_logger().info('Girando à esquerda...')
        time.sleep(duracao)
        self.parar()

    def parar(self):
        msg = Twist()  
        self.publisher_.publish(msg)
        self.get_logger().info('Parado.')
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = MoverCarrinho()
    node.mover_frente(velocidade=0.3, duracao=3.0)
    node.girar_esquerda(velocidade=0.5, duracao=2.0)
    node.mover_frente(velocidade=0.3, duracao=3.0)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
