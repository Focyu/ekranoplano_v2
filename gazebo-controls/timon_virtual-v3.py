#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
import curses
import math
import time
import random

RAMP_DURATION  = 3.0
INTERVAL       = 15.0
ALT_MIN        = 0.10
KFF_ALTURA     = 0.5
H_MIN_GIRO     = 0.35
A_OLA_MAX      = 0.25


class VirtualRudder(Node):
    def __init__(self):
        super().__init__('virtual_rudder')

        self.pub_alt        = self.create_publisher(Float64, '/setpoint/altura',      1)
        self.pub_pitch      = self.create_publisher(Float64, '/setpoint/pitch',       1)
        self.pub_yaw        = self.create_publisher(Float64, '/setpoint/yaw',         1)
        self.pub_turb       = self.create_publisher(Bool,    '/setpoint/turbulencia', 1)
        self.pub_heave      = self.create_publisher(Float64, '/olas/heave',           10)
        self.pub_pitch_rate = self.create_publisher(Float64, '/olas/pitch_rate',      10)

        self.h_actual   = 1.0
        self.psi_actual = 0.0
        self.create_subscription(Float64, '/estado/altura', self._cb_altura, 10)
        self.create_subscription(Float64, '/estado/yaw',    self._cb_yaw,    10)

        self.alt_m        = 1.0
        self.pitch_deg    = 0.0
        self.heading_deg  = 0.0
        self.step_heading = 5.0
        self.step_alt     = 0.10
        self.step_deg     = 0.10
        self.girando      = False

        # ── NUEVO: modo resorte alternable con R ─────────────────────
        self.resorte_enabled = False   # OFF por defecto (comportamiento v5)

        self.turb_enabled  = False
        self.ocean_enabled = False

        self.A_min, self.A_max = 0.10, 0.25
        self.T_min, self.T_max = 5.0,  10.0
        self.A_cur = 0.15;  self.T_cur = 7.5
        self._A_ef = self.A_cur;  self._T_ef = self.T_cur
        self.A_tgt = self.A_cur;  self.T_tgt = self.T_cur
        self.t0            = time.time()
        self.last_interval = time.time()
        self.ramp_start    = None

        self.create_timer(0.05, self.publish_olas)

    def _cb_altura(self, msg):
        self.h_actual = max(ALT_MIN, msg.data)

    def _cb_yaw(self, msg):
        self.psi_actual = self._normalizar_heading(math.degrees(msg.data))

    # ── Efecto resorte ───────────────────────────────────────────────
    def _aplicar_resorte(self):
        """Solo actúa si resorte_enabled=True y no se está girando."""
        if not self.resorte_enabled or self.girando:
            return
        error = self.psi_actual - self.heading_deg
        while error >  180.0: error -= 360.0
        while error < -180.0: error += 360.0
        self.heading_deg += error * 0.07
        self.heading_deg = self._normalizar_heading(self.heading_deg)

    # ── Modo joystick (v5): yaw vuelve a 0 al soltar ────────────────
    def _aplicar_joystick(self):
        """Solo actúa si resorte_enabled=False y no se está girando."""
        if self.resorte_enabled or self.girando:
            return
        # Decay suave hacia 0° (heading neutro)
        self.heading_deg *= 0.85
        if abs(self.heading_deg) < 0.1:
            self.heading_deg = 0.0

    def _altura_corregida(self):
        heading_rad  = math.radians(self.heading_deg)
        phi_estimado = min(abs(heading_rad) * 0.15, math.radians(10.0))
        delta_h = self.alt_m * KFF_ALTURA * (1.0 - math.cos(phi_estimado))
        return max(ALT_MIN, self.alt_m + delta_h)

    def _normalizar_heading(self, deg):
        while deg >  180.0: deg -= 360.0
        while deg < -180.0: deg += 360.0
        return deg

    def _giro_bloqueado(self):
        return self.h_actual < H_MIN_GIRO

    def _phi_max_estimado(self):
        semi_span = 0.3489
        zw        = 0.0505
        h_ef      = max(0.001, self.h_actual - A_OLA_MAX)
        h_libre   = max(0.001, h_ef - zw)
        sin_phi   = min(0.9999, h_libre / semi_span)
        phi_geo   = math.degrees(math.asin(sin_phi))
        return min(phi_geo * 0.60, 10.0)

    def _iniciar_transicion(self):
        self.A_cur = self._A_ef;  self.T_cur = self._T_ef
        self.A_tgt = random.uniform(self.A_min, self.A_max)
        self.T_tgt = random.uniform(self.T_min, self.T_max)
        self.ramp_start = time.time()

    def _interpolar(self, now):
        if self.ramp_start is None:
            return self.A_cur, self.T_cur
        alpha = (now - self.ramp_start) / RAMP_DURATION
        if alpha >= 1.0:
            self.A_cur = self.A_tgt;  self.T_cur = self.T_tgt
            self.ramp_start = None
            return self.A_cur, self.T_cur
        s = 0.5 * (1.0 - math.cos(math.pi * alpha))
        return (self.A_cur + s*(self.A_tgt - self.A_cur),
                self.T_cur + s*(self.T_tgt - self.T_cur))

    def publish_olas(self):
        now = time.time()
        if not self.ocean_enabled:
            self.pub_heave.publish(Float64(data=0.0))
            self.pub_pitch_rate.publish(Float64(data=0.0))
            return
        if self.ramp_start is None and (now - self.last_interval) > INTERVAL:
            self._iniciar_transicion()
            self.last_interval = now
        A, T = self._interpolar(now)
        self._A_ef = A;  self._T_ef = T
        omega = 2.0 * math.pi / T
        t = now - self.t0
        self.pub_heave.publish(Float64(data=float(A * math.cos(omega * t))))
        self.pub_pitch_rate.publish(Float64(data=float(-A*omega*math.sin(omega*t))))

    def publish_all(self):
        self.pub_alt.publish(Float64(data=float(self._altura_corregida())))
        self.pub_pitch.publish(Float64(data=math.radians(self.pitch_deg)))
        self.pub_yaw.publish(Float64(data=math.radians(self.heading_deg)))
        self.pub_turb.publish(Bool(data=self.turb_enabled))

    def shutdown_limpio(self):
        self.pub_heave.publish(Float64(data=0.0))
        self.pub_pitch_rate.publish(Float64(data=0.0))
        self.pub_turb.publish(Bool(data=False))
        self.pub_yaw.publish(Float64(data=math.radians(self.heading_deg)))
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.02)


def main(stdscr):
    rclpy.init()
    node = VirtualRudder()

    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.keypad(True)
    curses.start_color()
    curses.init_pair(1, curses.COLOR_RED,    curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_GREEN,  curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_CYAN,   curses.COLOR_BLACK)

    last_publish_time = time.time()
    publish_interval  = 0.1

    try:
        while rclpy.ok():
            c = stdscr.getch()
            tecla_pulsada = False
            node.girando  = False

            if c != -1:
                tecla_pulsada = True

                if c == curses.KEY_LEFT:
                    node.heading_deg -= node.step_heading
                    node.girando = True
                elif c == curses.KEY_RIGHT:
                    node.heading_deg += node.step_heading
                    node.girando = True

                elif c == curses.KEY_UP:        node.pitch_deg += node.step_deg
                elif c == curses.KEY_DOWN:      node.pitch_deg -= node.step_deg
                elif c in (ord('w'), ord('W')): node.alt_m     += node.step_alt
                elif c in (ord('s'), ord('S')): node.alt_m     -= node.step_alt

                # ── R alterna modo resorte ───────────────────────────
                elif c == ord('R'):
                    node.resorte_enabled = not node.resorte_enabled
                    # Al activar resorte: heading SP se alinea con heading real
                    # para evitar un salto brusco al activarlo
                    if node.resorte_enabled:
                        node.heading_deg = node.psi_actual

                elif c == ord(' '):
                    node.pitch_deg = 0.0
                    if node.resorte_enabled:
                        node.heading_deg = node.psi_actual  # centra en real
                    else:
                        node.heading_deg = 0.0              # centra en norte

                elif c in (ord('t'), ord('T')):
                    node.turb_enabled = not node.turb_enabled
                elif c in (ord('o'), ord('O')):
                    node.ocean_enabled = not node.ocean_enabled
                    node.t0 = node.last_interval = time.time()
                    node.ramp_start = None
                elif c in (ord('q'), ord('Q')):
                    break

                node.alt_m       = max(ALT_MIN, round(node.alt_m, 2))
                node.pitch_deg   = round(node.pitch_deg, 2)
                node.heading_deg = node._normalizar_heading(
                    round(node.heading_deg, 1))

            # ── Aplicar modo activo ──────────────────────────────────
            node._aplicar_resorte()   # solo actúa si resorte_enabled=True
            node._aplicar_joystick()  # solo actúa si resorte_enabled=False

            now = time.time()
            if (now - last_publish_time >= publish_interval) or tecla_pulsada:
                node.publish_all()
                last_publish_time = now

            rclpy.spin_once(node, timeout_sec=0.01)

            # ── Interfaz ─────────────────────────────────────────────
            stdscr.erase()
            modo_txt = "RESORTE (sigue heading real)" if node.resorte_enabled \
                       else "JOYSTICK (vuelve a 0°)"
            stdscr.addstr(0, 0, "=== TIMON VIRTUAL v6 ===  modo: ", curses.A_BOLD)
            stdscr.addstr(0, 33, modo_txt,
                curses.color_pair(4) if node.resorte_enabled
                else curses.color_pair(3))

            stdscr.addstr(1, 0, "[W/S] Altura  [↑↓] Pitch  [←→] Heading")
            stdscr.addstr(2, 0, "[R] Modo resorte/joystick  [SPC] Centrar  "
                                "[T] Turb  [O] Oceano  [Q] Salir")

            bloqueado = node._giro_bloqueado()
            phi_max   = node._phi_max_estimado()
            error_hdg = node._normalizar_heading(
                node.heading_deg - node.psi_actual)

            stdscr.addstr(4, 0, "--- VUELO ---")
            stdscr.addstr(5, 0,
                f"Altura SP  : {node.alt_m:.2f} m  "
                f"(corr {node._altura_corregida():.2f} m)  "
                f"h_real: {node.h_actual:.2f} m")
            stdscr.addstr(6, 0, f"Pitch      : {node.pitch_deg:.2f} deg")
            stdscr.addstr(7, 0,
                f"Heading SP : {node.heading_deg:.1f} deg  "
                f"real: {node.psi_actual:.1f} deg  "
                f"error: {error_hdg:+.1f} deg")

            if bloqueado:
                stdscr.addstr(8, 0,
                    f"Giro       : BLOQUEADO h={node.h_actual:.2f}m < {H_MIN_GIRO}m  "
                    f"↑ sube altura",
                    curses.color_pair(1) | curses.A_BOLD)
            elif phi_max < 3.0:
                stdscr.addstr(8, 0,
                    f"Giro       : LIMITADO  phi_max={phi_max:.1f}°",
                    curses.color_pair(2))
            else:
                stdscr.addstr(8, 0,
                    f"Giro       : OK  phi_max={phi_max:.1f}°",
                    curses.color_pair(3))

            turb_txt  = "ON" if node.turb_enabled else "OFF"
            ocean_txt = (
                f"ON  A:{node._A_ef:.2f}m T:{node._T_ef:.2f}s"
                + (" [ramp]" if node.ramp_start else "")
                if node.ocean_enabled else "OFF"
            )
            stdscr.addstr(10, 0, "--- PERTURBACIONES ---")
            stdscr.addstr(11, 0, f"Turbulencia: {turb_txt}")
            stdscr.addstr(12, 0, f"Oceano     : {ocean_txt}")
            stdscr.refresh()

    except curses.error:
        pass
    finally:
        node.shutdown_limpio()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    curses.wrapper(main)
