#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
import curses
import math
import time
import random

# Duración de la rampa de transición entre estados de mar (segundos)
RAMP_DURATION = 3.0
# Cada cuántos segundos cambia el estado de mar
INTERVAL = 15.0

# ── Parámetros de giro coordinado ──────────────────────────────────────────
# Relación bank/yaw: por cada grado de timón se piden N grados de alabeo
BANK_POR_YAW   = 10.0 / 15.0   # 10° de bank para 15° de timón
# Máximo bank absoluto permitido (se recortará además según altura)
ROLL_MAX_ABS   = 15.0           # [deg]
# Ganancia feedforward altura: sube el setpoint de h durante el giro
KFF_ALTURA     = 0.5            # h_sp_corr = h_sp * (1 + KFF * (1 - cos(phi)))
# Altura mínima de seguridad (nunca bajar de aquí)
ALT_MIN        = 0.10           # [m]


class VirtualRudder(Node):
    def __init__(self):
        super().__init__('virtual_rudder')

        # --- Publicadores de control ---
        self.pub_alt   = self.create_publisher(Float64, '/setpoint/altura',      1)
        self.pub_pitch = self.create_publisher(Float64, '/setpoint/pitch',       1)
        self.pub_yaw   = self.create_publisher(Float64, '/setpoint/yaw',         1)
        self.pub_roll  = self.create_publisher(Float64, '/setpoint/roll',        1)  # NUEVO
        self.pub_turb  = self.create_publisher(Bool,    '/setpoint/turbulencia', 1)

        # --- Publicadores de olas ---
        self.pub_heave      = self.create_publisher(Float64, '/olas/heave',      10)
        self.pub_pitch_rate = self.create_publisher(Float64, '/olas/pitch_rate', 10)

        # --- Suscriptor de altura real (para limitador de roll) ---
        self.h_actual = 1.0   # valor por defecto hasta recibir primer mensaje
        self.create_subscription(Float64, '/estado/altura',
                                 self._cb_altura, 10)

        # --- Estado del timón ---
        self.alt_m         = 1.0
        self.pitch_deg     = 0.0
        self.yaw_deg       = 0.0
        self.roll_deg      = 0.0   # NUEVO — calculado automáticamente
        self.turb_enabled  = False
        self.ocean_enabled = False

        self.step_alt = 0.10
        self.step_deg = 0.10

        # --- Parámetros de olas con ramp ---
        self.A_min, self.A_max = 0.10, 0.25
        self.T_min, self.T_max = 5.0, 10.0

        self.A_cur = 0.15
        self.T_cur = 7.5
        self._A_ef = self.A_cur
        self._T_ef = self.T_cur
        self.A_tgt = self.A_cur
        self.T_tgt = self.T_cur

        self.t0            = time.time()
        self.last_interval = time.time()
        self.ramp_start    = None

        # Timer olas a 20 Hz
        self.create_timer(0.05, self.publish_olas)

    # ── Callback altura real ─────────────────────────────────────────────────
    def _cb_altura(self, msg):
        self.h_actual = max(ALT_MIN, msg.data)

    # ── Cálculo de roll máximo según altura real ─────────────────────────────
    def _roll_max(self):
        """Roll máximo permitido [deg] en función de la altura medida."""
        # A 0.55 m → ~10°  |  A 1.0 m → ~11.8°  |  A 2.0 m → ~15°
        roll_por_h = 8.0 + self.h_actual * 3.5
        return min(roll_por_h, ROLL_MAX_ABS)

    # ── Cálculo de roll coordinado ───────────────────────────────────────────
    def _calcular_roll(self, yaw_deg):
        """Devuelve el setpoint de bank coordinado con el yaw pedido."""
        roll_raw = yaw_deg * BANK_POR_YAW
        limit    = self._roll_max()
        return max(-limit, min(limit, roll_raw))

    # ── Cálculo de altura corregida por feedforward ──────────────────────────
    def _altura_corregida(self):
        """Sube ligeramente h_sp durante el giro para compensar pérdida de sustentación."""
        phi_rad  = math.radians(self.roll_deg)
        delta_h  = self.alt_m * KFF_ALTURA * (1.0 - math.cos(phi_rad))
        return self.alt_m + delta_h

    # ── Lógica de olas ───────────────────────────────────────────────────────
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
        if self.ramp_start is None and (now - self.last_interval) > INTERVAL:
            self._iniciar_transicion()
            self.last_interval = now
        A, T = self._interpolar(now)
        self._A_ef = A
        self._T_ef = T
        omega = 2.0 * math.pi / T
        t     = now - self.t0
        self.pub_heave.publish(Float64(data=float(A * math.cos(omega * t))))
        self.pub_pitch_rate.publish(Float64(data=float(-A * omega * math.sin(omega * t))))

    def shutdown_limpio(self):
        self.pub_heave.publish(Float64(data=0.0))
        self.pub_pitch_rate.publish(Float64(data=0.0))
        self.pub_turb.publish(Bool(data=False))
        self.pub_roll.publish(Float64(data=0.0))
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.02)

    # ── Publicación principal ────────────────────────────────────────────────
    def publish_all(self):
        h_corr = self._altura_corregida()
        self.pub_alt.publish(Float64(data=float(h_corr)))
        self.pub_pitch.publish(Float64(data=math.radians(self.pitch_deg)))
        self.pub_yaw.publish(Float64(data=math.radians(self.yaw_deg)))
        self.pub_roll.publish(Float64(data=math.radians(self.roll_deg)))
        self.pub_turb.publish(Bool(data=self.turb_enabled))


def main(stdscr):
    rclpy.init()
    node = VirtualRudder()

    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.keypad(True)

    last_publish_time = time.time()
    publish_interval  = 0.1   # 10 Hz

    try:
        while rclpy.ok():
            c = stdscr.getch()
            tecla_pulsada = False

            # Yaw y roll vuelven a 0 si no se pulsa nada (modo joystick)
            node.yaw_deg  = 0.0
            node.roll_deg = 0.0

            if c != -1:
                tecla_pulsada = True

                if c == curses.KEY_LEFT:
                    node.yaw_deg  = -15.0
                    node.roll_deg = node._calcular_roll(-15.0)   # bank coordinado

                elif c == curses.KEY_RIGHT:
                    node.yaw_deg  =  15.0
                    node.roll_deg = node._calcular_roll(15.0)    # bank coordinado

                # Pitch y Altura: modo "trim" (se quedan donde los dejes)
                elif c == curses.KEY_UP:            node.pitch_deg += node.step_deg
                elif c == curses.KEY_DOWN:          node.pitch_deg -= node.step_deg
                elif c in (ord('w'), ord('W')):     node.alt_m     += node.step_alt
                elif c in (ord('s'), ord('S')):     node.alt_m     -= node.step_alt

                # Espacio: resetea pitch
                elif c == ord(' '):
                    node.pitch_deg = 0.0

                # Perturbaciones
                elif c in (ord('t'), ord('T')):
                    node.turb_enabled = not node.turb_enabled
                elif c in (ord('o'), ord('O')):
                    node.ocean_enabled = not node.ocean_enabled
                    node.t0            = time.time()
                    node.last_interval = time.time()
                    node.ramp_start    = None
                elif c in (ord('q'), ord('Q')):
                    break

                node.alt_m     = max(ALT_MIN, round(node.alt_m, 2))
                node.pitch_deg = round(node.pitch_deg, 2)

            current_time = time.time()
            if (current_time - last_publish_time >= publish_interval) or tecla_pulsada:
                node.publish_all()
                last_publish_time = current_time

            rclpy.spin_once(node, timeout_sec=0.01)

            # ── Interfaz ────────────────────────────────────────────────────
            stdscr.erase()
            stdscr.addstr(0, 0, "=== TIMON VIRTUAL v4 (giro coordinado) ===")
            stdscr.addstr(1, 0, "[W/S] Altura  [UP/DOWN] Pitch  [LEFT/RIGHT] Yaw+Roll  [SPC] Centrar pitch")
            stdscr.addstr(2, 0, "[T] Turbulencia  [O] Oceano  [Q] Salir")

            roll_lim = node._roll_max()
            stdscr.addstr(4, 0, "--- VUELO ---")
            stdscr.addstr(5, 0, f"Altura SP  : {node.alt_m:.2f} m  (corr→{node._altura_corregida():.2f} m)  h_real: {node.h_actual:.2f} m")
            stdscr.addstr(6, 0, f"Pitch      : {node.pitch_deg:.2f} deg")
            stdscr.addstr(7, 0, f"Yaw        : {node.yaw_deg:.2f} deg")
            stdscr.addstr(8, 0, f"Roll       : {node.roll_deg:.2f} deg  (lim ±{roll_lim:.1f}°  @h={node.h_actual:.2f}m)")

            turb_txt = "ON" if node.turb_enabled else "OFF"
            if node.ocean_enabled:
                ramp_txt  = f" [ramp {RAMP_DURATION}s activa]" if node.ramp_start else ""
                ocean_txt = f"ON  A:{node._A_ef:.2f}m T:{node._T_ef:.2f}s{ramp_txt}"
            else:
                ocean_txt = "OFF"

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
