#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Bool
import curses
import math
import time

class JoyToEkranoplano(Node):
    def __init__(self):
        super().__init__('joy_to_ekranoplano')
        
        # Suscriptor al mando
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Publicadores hacia Simulink (QoS = 1)
        self.pub_alt = self.create_publisher(Float64, '/setpoint/altura', 1)

        self.pub_yaw = self.create_publisher(Float64, '/setpoint/yaw', 1)
        self.pub_turb = self.create_publisher(Bool, '/setpoint/turbulencia', 1)

        # Variables de estado
        self.alt_m = 1.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.turb_enabled = False

        # Pasos discretos
        self.step_alt = 0.10
        self.step_deg = 0.10

        # Control de estado de botones (para no registrar "múltiples clicks" al mantener pulsado)
        self.last_buttons = [0] * 15
        self.last_axes = [0.0] * 8
        
        # Bandera para saber si hubo un cambio y debemos actualizar la pantalla
        self.data_changed = True

    def joy_callback(self, msg):
        """
        Mapeo Xbox One Linux:
        Botones (msg.buttons):
            0: A (Turbulencia)
            1: B (Centrar)
            2: X (Disminuir Pitch - Morro abajo)
            3: Y (Aumentar Pitch - Morro arriba)
            4: LB (Disminuir Yaw - Girar izquierda)
            5: RB (Aumentar Yaw - Girar derecha)
        Ejes (msg.axes):
            7: D-Pad Arriba/Abajo (+1.0 Arriba, -1.0 Abajo) (Altura)
        """
        try:
            changed = False

            # --- YAW (LB y RB) ---
            if msg.buttons[4] == 1 and self.last_buttons[4] == 0:  # LB
                self.yaw_deg += self.step_deg
                changed = True
            elif msg.buttons[5] == 1 and self.last_buttons[5] == 0:  # RB
                self.yaw_deg -= self.step_deg
                changed = True

            # --- PITCH (Y y X) ---
            if msg.buttons[3] == 1 and self.last_buttons[3] == 0:  # Y
                self.pitch_deg += self.step_deg
                changed = True
            elif msg.buttons[2] == 1 and self.last_buttons[2] == 0:  # X
                self.pitch_deg -= self.step_deg
                changed = True

            # --- ALTURA (D-Pad Eje 7) ---
            # El eje del D-Pad marca +1.0 o -1.0. Solo aplicamos el cambio cuando pasa de 0 a 1 o -1
            if msg.axes[7] > 0.5 and self.last_axes[7] <= 0.5:
                self.alt_m += self.step_alt
                changed = True
            elif msg.axes[7] < -0.5 and self.last_axes[7] >= -0.5:
                self.alt_m -= self.step_alt
                changed = True

            # --- CENTRAR (Botón B) ---
            if msg.buttons[1] == 1 and self.last_buttons[1] == 0:
                self.pitch_deg = 0.0
                self.yaw_deg = 0.0
                changed = True

            # --- TURBULENCIA (Botón A) ---
            if msg.buttons[0] == 1 and self.last_buttons[0] == 0:
                self.turb_enabled = not self.turb_enabled
                changed = True

            # Guardar estado actual para la siguiente iteración
            self.last_buttons = list(msg.buttons)
            self.last_axes = list(msg.axes)

            if changed:
                # Redondeo para evitar decimales extraños
                self.alt_m = round(self.alt_m, 2)
                self.pitch_deg = round(self.pitch_deg, 2)
                self.yaw_deg = round(self.yaw_deg, 2)
                
                # Publicar a ROS 2
                self.publish_all()
                self.data_changed = True

        except IndexError:
            pass

    def publish_all(self):
        m_alt = Float64()
        m_alt.data = float(self.alt_m)
        self.pub_alt.publish(m_alt)
        
        m_pitch = Float64()
        m_pitch.data = math.radians(self.pitch_deg)
        self.pub_pitch.publish(m_pitch)
        
        m_yaw = Float64()
        m_yaw.data = math.radians(self.yaw_deg)
        self.pub_yaw.publish(m_yaw)
        
        m_turb = Bool()
        m_turb.data = self.turb_enabled
        self.pub_turb.publish(m_turb)

def main(stdscr):
    # Inicializar ROS 2
    rclpy.init()
    node = JoyToEkranoplano()

    # Configurar interfaz de terminal
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.keypad(True)
    curses.curs_set(0)  # Ocultar el cursor parpadeante

    # Dibujar la pantalla inicial
    node.publish_all()

    try:
        while rclpy.ok():
            # ROS 2 recibe los mensajes del mando y llama a joy_callback()
            rclpy.spin_once(node, timeout_sec=0.05)

            # Para poder salir con la tecla Q desde el PC
            c = stdscr.getch()
            if c in (ord('q'), ord('Q')):
                break

            # Si hubo un cambio desde el mando, actualizamos la pantalla
            if node.data_changed:
                try:
                    stdscr.erase()
                    stdscr.addstr(0, 0, "=== TIMON XBOX (EKRANOPLANO) ===")
                    stdscr.addstr(1, 0, f"[D-Pad Arr/Aba] Altura (+/- {node.step_alt} m)")
                    stdscr.addstr(2, 0, f"[Botón Y / X]   Pitch  (+/- {node.step_deg} deg)")
                    stdscr.addstr(3, 0, f"[Bumper LB/RB]  Yaw    (+/- {node.step_deg} deg)")
                    stdscr.addstr(4, 0, "[Botón B]       Centrar Ángulos a 0.0")
                    stdscr.addstr(5, 0, "[Botón A]       Turbulencia On/Off")
                    stdscr.addstr(6, 0, "[Tecla Q PC]    Salir del programa")
                    
                    stdscr.addstr(8, 0, "--- ESTADO ACTUAL ---")
                    stdscr.addstr(9, 0,  f"Altura: {node.alt_m:.2f} m")
                    stdscr.addstr(10, 0, f"Pitch : {node.pitch_deg:.2f} deg  (ROS: {math.radians(node.pitch_deg):.4f} rad)")
                    stdscr.addstr(11, 0, f"Yaw   : {node.yaw_deg:.2f} deg  (ROS: {math.radians(node.yaw_deg):.4f} rad)")
                    
                    turb_text = "ENCENDIDA" if node.turb_enabled else "APAGADA"
                    stdscr.addstr(12, 0, f"Turbulencia : {turb_text}")
                    
                    stdscr.refresh()
                    node.data_changed = False
                except curses.error:
                    pass

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    curses.wrapper(main)
