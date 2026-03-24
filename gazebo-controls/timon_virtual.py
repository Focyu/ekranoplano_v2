#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
import curses
import math
import time

class VirtualRudder(Node):
    def __init__(self):
        super().__init__('virtual_rudder')
        # QoS en 1
        self.pub_alt = self.create_publisher(Float64, '/setpoint/altura', 1)

        self.pub_yaw = self.create_publisher(Float64, '/setpoint/yaw', 1)
        # Nuevo publicador para turbulencia
        self.pub_turb = self.create_publisher(Bool, '/setpoint/turbulencia', 1)

        self.alt_m = 1.0

        self.yaw_deg = 0.0
        self.turb_enabled = False  # Empieza apagada

        self.step_alt = 0.10  
        self.step_deg = 0.10  

    def publish_all(self):
        m_alt = Float64()
        m_alt.data = float(self.alt_m)
        self.pub_alt.publish(m_alt)
        
        
        m_yaw = Float64()
        m_yaw.data = math.radians(self.yaw_deg)
        self.pub_yaw.publish(m_yaw)

        # Publicar estado de turbulencia
        m_turb = Bool()
        m_turb.data = self.turb_enabled
        self.pub_turb.publish(m_turb)

def main(stdscr):
    rclpy.init()
    node = VirtualRudder()

    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.keypad(True)

    last_publish_time = time.time()
    publish_interval = 0.1  # 10 Hz

    try:
        while rclpy.ok():
            c = stdscr.getch()
            tecla_pulsada = False

            if c != -1:
                tecla_pulsada = True
                
                if c == curses.KEY_LEFT:
                    node.yaw_deg -= node.step_deg
                elif c == curses.KEY_RIGHT:
                    node.yaw_deg += node.step_deg

                elif c in (ord('w'), ord('W')):
                    node.alt_m += node.step_alt
                elif c in (ord('s'), ord('S')):
                    node.alt_m -= node.step_alt
                elif c == ord(' '):
                    node.yaw_deg = 0.0

                elif c in (ord('t'), ord('T')):
                    # Cambia el estado de la turbulencia (On/Off)
                    node.turb_enabled = not node.turb_enabled
                elif c in (ord('q'), ord('Q')):
                    break
                    
                node.alt_m = round(node.alt_m, 2)

                node.yaw_deg = round(node.yaw_deg, 2)

            current_time = time.time()
            if (current_time - last_publish_time >= publish_interval) or tecla_pulsada:
                node.publish_all()
                last_publish_time = current_time
            
            rclpy.spin_once(node, timeout_sec=0.01)

            # Interfaz compacta
            stdscr.erase()
            stdscr.addstr(0, 0, "=== TIMON VIRTUAL ===")
            stdscr.addstr(1, 0, f"[W/S] Altura (+/- {node.step_alt} m) |  [←/→] Yaw (+/- {node.step_deg} deg)")
            stdscr.addstr(2, 0, "[ESPACIO] Centrar | [T] Turbulencia On/Off | [Q] Salir")
            
            stdscr.addstr(4, 0, "--- ESTADO ACTUAL ---")
            stdscr.addstr(5, 0, f"Altura: {node.alt_m:.2f} m")

            stdscr.addstr(7, 0, f"Yaw   : {node.yaw_deg:.2f} deg  (ROS: {math.radians(node.yaw_deg):.4f} rad)")
            
            turb_text = "ENCENDIDA (Cuidado)" if node.turb_enabled else "APAGADA (Calma)"
            stdscr.addstr(8, 0, f"Turbulencia : {turb_text}")
            
            stdscr.refresh()

    except curses.error:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    curses.wrapper(main)
