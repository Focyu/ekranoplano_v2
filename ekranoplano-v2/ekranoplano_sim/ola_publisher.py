#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time

class OlaPublisher(Node):
    def __init__(self):
        super().__init__('ola_publisher')
        self.pub_heave      = self.create_publisher(Float64, '/olas/heave', 10)
        self.pub_pitch_rate = self.create_publisher(Float64, '/olas/pitch_rate', 10)
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

        # Sea State 2: A=0.20m, T=6.5s
        self.A     = 0.20
        self.T     = 6.5
        self.omega = 2 * math.pi / self.T
        self.t0    = time.time()
        self.get_logger().info('🌊 Ola publisher iniciado — Sea State 2 (A=0.20m, T=6.5s)')

    def tick(self):
        t = time.time() - self.t0

        # Heave: desplazamiento vertical sinusoidal
        heave = self.A * math.cos(self.omega * t)

        # Pitch rate: derivada del heave (velocidad angular de cabeceo)
        pitch_rate = -self.A * self.omega * math.sin(self.omega * t)

        msg_h = Float64()
        msg_h.data = float(heave)
        self.pub_heave.publish(msg_h)

        msg_p = Float64()
        msg_p.data = float(pitch_rate)
        self.pub_pitch_rate.publish(msg_p)

def main():
    rclpy.init()
    node = OlaPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

