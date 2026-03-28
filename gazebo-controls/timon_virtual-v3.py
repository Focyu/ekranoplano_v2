#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, Int64
import curses
import math
import time
import random

# Duración de la rampa de transición entre estados de mar (segundos)
RAMP_DURATION = 3.0
# Cada cuántos segundos cambia el estado de mar
INTERVAL = 15.0

class VirtualRudder(Node):
    def __init__(self):
        super().__init__('virtual_rudder')

        # --- Publicadores de control ---
        self.pub_alt   = self.create_publisher(Float64, '/setpoint/altura', 1)
        self.pub_pitch = self.create_publisher(Float64, '/setpoint/pitch', 1)
        self.pub_yaw   = self.create_publisher(Float64, '/setpoint/yaw', 1)
        self.pub_turb  = self.create_publisher(Bool,    '/setpoint/turbulencia', 1)

        # --- Waypoints y modo ---
        self.pub_wp_x  = self.create_publisher(Float64, '/setpoint/waypoint_x', 1)
        self.pub_wp_y  = self.create_publisher(Float64, '/setpoint/waypoint_y', 1)
        self.pub_mode  = self.create_publisher(Int64,   '/setpoint/mode', 1)

        # --- Publicadores de olas ---
        self.pub_heave      = self.create_publisher(Float64, '/olas/heave', 10)
        self.pub_pitch_rate = self.create_publisher(Float64, '/olas/pitch_rate', 10)

        # --- Estado del timón ---
        self.alt_m         = 1.0
        self.pitch_deg     = 0.0
        self.yaw_deg       = 0.0
        self.turb_enabled  = False
        self.ocean_enabled = False

        # --- Waypoints ---
        self.wp_x    = 0.0
        self.wp_y    = 0.0
        self.wp_step = 5.0
        self.mode    = 0  # 0 = yaw manual, 1 = waypoint

        self.step_alt = 0.10
        self.step_deg = 0.10

        # --- Parámetros de olas con ramp ---
        self.A_min, self.A_max = 0.1, 1.5
        self.T_min, self.T_max = 2.0, 10.0

        self.A_cur = 0.5
        self.T_cur = 5.0
        self._A_ef = self.A_cur
        self._T_ef = self.T_cur
        self.A_tgt = self.A_cur
        self.T_tgt = self.T_cur

        self.t0               = time.time()
        self.last_interval    = time.time()
        self.ramp_start       = None

        # Timer olas a 20 Hz
        self.create_timer(0.05, self.publish_olas)

    # ── Lógica de olas ──────────────────────────────────────────────────────

    def _iniciar_transicion(self):
        self.A_cur = self._A_ef
        self.T_cur = self._T_ef
        self.A_tgt = random.uniform(self.A_min, self.A_max)
        self.T_tgt = random.uniform(self.T_min, self.T_max)
        self.ramp_start = time.time()
        self.get_logger().info(
            f'🔄 Nuevo mar -> A:{self.A_tgt:.2f}m T:{self.T_tgt:.2f}s '
            f'(ramp {RAMP_DURATION}s)'
        )

    def _interpolar(self, now):
        if self.ramp_start is None:
            return self.A_cur, self.T_cur
        alpha = (now - self.ramp_start) / RAMP_DURATION
        if alpha >= 1.0:
            self.A_cur = self.A_tgt
            self.T_cur = self.T_tgt
            self.ramp_start = None
            return self.A_cur, self.T_cur
        s = 0.5 * (1.0 - math.cos(math.pi * alpha))
        return (self.A_cur + s * (self.A_tgt - self.A_cur),
                self.T_cur + s * (self.T_tgt - self.T_cur))

    def publish_olas(self):
        now = time.time()

        if not self.ocean_enabled:
            self.pub_heave.publish(Float64(data=0.0))
            self.pub_pitch_rate.publish(Float64(data=0.0))
            return

        # Cambio periódico solo si el océano está activo
        if self.ramp_start is None and (now - self.last_interval) > INTERVAL:
            self._iniciar_transicion()
            self.last_interval = now

        A, T = self._interpolar(now)
        self._A_ef = A
        self._T_ef = T
        omega = 2.0 * math.pi / T
        t = now - self.t0

        self.pub_heave.publish(Float64(data=float(A * math.cos(omega * t))))
        self.pub_pitch_rate.publish(Float64(data=float(-A * omega * math.sin(omega * t))))

    # ── Control general ─────────────────────────────────────────────────────

    def publish_all(self):
        self.pub_alt.publish(Float64(data=float(self.alt_m)))
        self.pub_pitch.publish(Float64(data=math.radians(self.pitch_deg)))
        self.pub_yaw.publish(Float64(data=math.radians(self.yaw_deg)))
        self.pub_turb.publish(Bool(data=self.turb_enabled))
        self.pub_wp_x.publish(Float64(data=float(self.wp_x)))
        self.pub_wp_y.publish(Float64(data=float(self.wp_y)))
        self.pub_mode.publish(Int64(data=int(self.mode)))


def main(stdscr):
    rclpy.init()
    node = VirtualRudder()

    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.keypad(True)

    last_publish_time = time.time()
    publish_interval  = 0.1  # 10 Hz

    try:
        while rclpy.ok():
            c = stdscr.getch()
            tecla_pulsada = False

            if c != -1:
                tecla_pulsada = True

                if   c == curses.KEY_LEFT:          node.yaw_deg   -= node.step_deg
                elif c == curses.KEY_RIGHT:         node.yaw_deg   += node.step_deg
                elif c == curses.KEY_UP:            node.pitch_deg += node.step_deg
                elif c == curses.KEY_DOWN:          node.pitch_deg -= node.step_deg
                elif c in (ord('w'), ord('W')):     node.alt_m     += node.step_alt
                elif c in (ord('s'), ord('S')):     node.alt_m     -= node.step_alt
                elif c == ord(' '):
                    node.yaw_deg   = 0.0
                    node.pitch_deg = 0.0
                elif c in (ord('t'), ord('T')):
                    node.turb_enabled = not node.turb_enabled
                elif c in (ord('o'), ord('O')):
                    node.ocean_enabled = not node.ocean_enabled
                    node.t0 = time.time()          # reinicia fase de ola
                    node.last_interval = time.time()
                    node.ramp_start = None
                elif c in (ord('i'), ord('I')): node.wp_y += node.wp_step
                elif c in (ord('k'), ord('K')): node.wp_y -= node.wp_step
                elif c in (ord('l'), ord('L')): node.wp_x += node.wp_step
                elif c in (ord('j'), ord('J')): node.wp_x -= node.wp_step
                elif c in (ord('r'), ord('R')):
                    node.wp_x = 0.0
                    node.wp_y = 0.0
                elif c in (ord('m'), ord('M')):
                    node.mode = 1 if node.mode == 0 else 0
                elif c in (ord('q'), ord('Q')):
                    break

                node.alt_m     = max(0.1, round(node.alt_m, 2))
                node.pitch_deg = round(node.pitch_deg, 2)
                node.yaw_deg   = round(node.yaw_deg, 2)
                node.wp_x      = round(node.wp_x, 1)
                node.wp_y      = round(node.wp_y, 1)

            current_time = time.time()
            if (current_time - last_publish_time >= publish_interval) or tecla_pulsada:
                node.publish_all()
                last_publish_time = current_time

            rclpy.spin_once(node, timeout_sec=0.01)

            # --- Interfaz ---
            stdscr.erase()
            stdscr.addstr(0, 0, "=== TIMON VIRTUAL v2 ===")
            stdscr.addstr(1, 0, "[W/S] Altura  [UP/DOWN] Pitch  [LEFT/RIGHT] Yaw  [SPC] Centrar")
            stdscr.addstr(2, 0, "[T] Turbulencia  [O] Oceano  [M] Modo  [Q] Salir")
            stdscr.addstr(3, 0, "[I/K] WP Norte/Sur  [J/L] WP Oeste/Este  [R] Reset WP")

            stdscr.addstr(5, 0, "--- VUELO ---")
            stdscr.addstr(6, 0, f"Altura : {node.alt_m:.2f} m")
            stdscr.addstr(7, 0, f"Pitch  : {node.pitch_deg:.2f} deg  ({math.radians(node.pitch_deg):.4f} rad)")
            stdscr.addstr(8, 0, f"Yaw    : {node.yaw_deg:.2f} deg  ({math.radians(node.yaw_deg):.4f} rad)")

            mode_txt  = "WAYPOINT (auto-yaw)" if node.mode == 1 else "YAW MANUAL"
            turb_txt  = "ON" if node.turb_enabled else "OFF"

            # Mostrar parámetros actuales de ola
            if node.ocean_enabled:
                ramp_txt = f" [ramp {RAMP_DURATION}s activa]" if node.ramp_start else ""
                ocean_txt = f"ON  A:{node._A_ef:.2f}m T:{node._T_ef:.2f}s{ramp_txt}"
            else:
                ocean_txt = "OFF"

            stdscr.addstr(10, 0, "--- MODO ---")
            stdscr.addstr(11, 0, f"Modo      : {mode_txt}")
            stdscr.addstr(12, 0, f"Waypoint X: {node.wp_x:.1f} m")
            stdscr.addstr(13, 0, f"Waypoint Y: {node.wp_y:.1f} m")

            stdscr.addstr(15, 0, "--- PERTURBACIONES ---")
            stdscr.addstr(16, 0, f"Turbulencia: {turb_txt}")
            stdscr.addstr(17, 0, f"Oceano     : {ocean_txt}")

            stdscr.refresh()

    except curses.error:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    curses.wrapper(main)
