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

        # --- Publicadores de control ---
        self.pub_alt   = self.create_publisher(Float64, '/setpoint/altura', 1)
        self.pub_pitch = self.create_publisher(Float64, '/setpoint/pitch', 1)
        self.pub_yaw   = self.create_publisher(Float64, '/setpoint/yaw', 1)
        self.pub_turb  = self.create_publisher(Bool,    '/setpoint/turbulencia', 1)

        # --- Publicadores de olas ---
        self.pub_heave      = self.create_publisher(Float64, '/olas/heave', 10)
        self.pub_pitch_rate = self.create_publisher(Float64, '/olas/pitch_rate', 10)

        # --- Estado del timón ---
        self.alt_m       = 1.0
        self.pitch_deg   = 0.0
        self.yaw_deg     = 0.0
        self.turb_enabled = False
        self.ocean_enabled = False

        self.step_alt = 0.10
        self.step_deg = 0.10

        # --- Parámetros de olas (Sea State 2) ---
        self.A     = 0.10          # Amplitud (m)
        self.T     = 6.5           # Periodo (s)
        self.omega = 2 * math.pi / self.T
        self.t0    = time.time()

        # --- Timer para olas a 20 Hz ---
        self.create_timer(0.05, self.publish_olas)

    def publish_olas(self):
        if not self.ocean_enabled:
            # Publica cero si el océano está apagado
            self.pub_heave.publish(Float64(data=0.0))
            self.pub_pitch_rate.publish(Float64(data=0.0))
            return

        t = time.time() - self.t0
        heave      =  self.A * math.cos(self.omega * t)
        pitch_rate = -self.A * self.omega * math.sin(self.omega * t)

        self.pub_heave.publish(Float64(data=float(heave)))
        self.pub_pitch_rate.publish(Float64(data=float(pitch_rate)))

    def publish_all(self):
        self.pub_alt.publish(Float64(data=float(self.alt_m)))
        self.pub_pitch.publish(Float64(data=math.radians(self.pitch_deg)))
        self.pub_yaw.publish(Float64(data=math.radians(self.yaw_deg)))
        self.pub_turb.publish(Bool(data=self.turb_enabled))


def main(stdscr):
    rclpy.init()
    node = VirtualRudder()

    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.keypad(True)

    last_publish_time = time.time()
    publish_interval  = 0.1  # 10 Hz para el timón

    try:
        while rclpy.ok():
            c = stdscr.getch()
            tecla_pulsada = False

            if c != -1:
                tecla_pulsada = True

                if   c == curses.KEY_LEFT:              node.yaw_deg   -= node.step_deg
                elif c == curses.KEY_RIGHT:             node.yaw_deg   += node.step_deg
                elif c == curses.KEY_UP:                node.pitch_deg += node.step_deg
                elif c == curses.KEY_DOWN:              node.pitch_deg -= node.step_deg
                elif c in (ord('w'), ord('W')):         node.alt_m     += node.step_alt
                elif c in (ord('s'), ord('S')):         node.alt_m     -= node.step_alt
                elif c == ord(' '):
                    node.yaw_deg   = 0.0
                    node.pitch_deg = 0.0
                elif c in (ord('t'), ord('T')):
                    node.turb_enabled = not node.turb_enabled
                elif c in (ord('o'), ord('O')):
                    node.ocean_enabled = not node.ocean_enabled
                    node.t0 = time.time()   # reinicia la fase de la ola
                elif c in (ord('q'), ord('Q')):
                    break

                node.alt_m     = max(0.1, round(node.alt_m, 2))
                node.pitch_deg = round(node.pitch_deg, 2)
                node.yaw_deg   = round(node.yaw_deg, 2)

            current_time = time.time()
            if (current_time - last_publish_time >= publish_interval) or tecla_pulsada:
                node.publish_all()
                last_publish_time = current_time

            rclpy.spin_once(node, timeout_sec=0.01)

            # --- Interfaz ---
            stdscr.erase()
            stdscr.addstr(0, 0, "=== TIMON VIRTUAL ===")
            stdscr.addstr(1, 0, f"[W/S] Altura  [↑/↓] Pitch  [←/→] Yaw  [ESPACIO] Centrar")
            stdscr.addstr(2, 0, f"[T] Turbulencia On/Off  [O] Océano On/Off  [Q] Salir")

            stdscr.addstr(4, 0, "--- ESTADO ---")
            stdscr.addstr(5, 0, f"Altura : {node.alt_m:.2f} m")
            stdscr.addstr(6, 0, f"Pitch  : {node.pitch_deg:.2f} deg  ({math.radians(node.pitch_deg):.4f} rad)")
            stdscr.addstr(7, 0, f"Yaw    : {node.yaw_deg:.2f} deg  ({math.radians(node.yaw_deg):.4f} rad)")

            turb_txt  = "✅ ENCENDIDA" if node.turb_enabled  else "⛔ APAGADA"
            ocean_txt = "✅ ENCENDIDO (A=0.10m, T=6.5s)" if node.ocean_enabled else "⛔ APAGADO"
            stdscr.addstr(8, 0,  f"Turbulencia : {turb_txt}")
            stdscr.addstr(9, 0,  f"Océano      : {ocean_txt}")

            stdscr.refresh()

    except curses.error:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    curses.wrapper(main)
