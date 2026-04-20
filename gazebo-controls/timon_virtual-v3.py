#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
import curses
import math
import time
import random

# --- Parámetros de Configuración ---
RAMP_DURATION = 3.0
INTERVAL      = 15.0
BANK_POR_YAW  = 6.0 / 15.0 
ROLL_MAX_ABS  = 15.0
KFF_ALTURA    = 0.10
ALT_MIN       = 0.10
YAW_STEP      = 1.5
YAW_MAX       = 45.0
YAW_HOLD_TIME = 0.50
K_PITCH_FF    = 0.8  

class VirtualRudder(Node):
    def __init__(self):
        super().__init__('virtual_rudder')

        # Publicadores
        self.pub_alt   = self.create_publisher(Float64, '/setpoint/altura',     1)
        self.pub_pitch = self.create_publisher(Float64, '/setpoint/pitch',      1)
        self.pub_yaw   = self.create_publisher(Float64, '/setpoint/yaw',        1)
        self.pub_roll  = self.create_publisher(Float64, '/setpoint/roll',       1)
        self.pub_turb  = self.create_publisher(Bool,    '/setpoint/turbulencia', 1)
        self.pub_heave      = self.create_publisher(Float64, '/olas/heave',      10)
        self.pub_pitch_rate = self.create_publisher(Float64, '/olas/pitch_rate', 10)

        # Estado y Feedback
        self.h_actual   = 1.0
        self.psi_actual = 0.0
        self.create_subscription(Float64, '/estado/altura', self._cb_altura, 10)
        self.create_subscription(Float64, '/estado/yaw',    self._cb_yaw,    10)

        self.alt_m         = 1.0
        self.pitch_deg     = 0.0
        self.turb_enabled  = False
        self.ocean_enabled = False
        self.step_alt      = 0.10
        self.step_deg      = 0.20

        self.yaw_acum       = 0.0
        self.yaw_active     = 0.0
        self.roll_active    = 0.0
        self.yaw_hold_until = 0.0

        # Olas
        self.A_min, self.A_max = 0.10, 0.25
        self.T_min, self.T_max = 5.0, 10.0
        self.A_cur = 0.15;  self.T_cur = 7.5
        self._A_ef = self.A_cur;  self._T_ef = self.T_cur
        self.A_tgt = self.A_cur;  self.T_tgt = self.T_cur
        self.t0            = time.time()
        self.last_interval = time.time()
        self.ramp_start    = None

        self.create_timer(0.05, self.publish_olas)

    def _cb_altura(self, msg): self.h_actual = msg.data
    def _cb_yaw(self, msg):    self.psi_actual = math.degrees(msg.data)

    def _roll_max(self):
        return min(8.0 + self.h_actual * 2.5, ROLL_MAX_ABS)

    def _altura_corregida(self):
        phi_rad = math.radians(self.roll_active)
        return self.alt_m + (self.alt_m * KFF_ALTURA * (1.0 - math.cos(phi_rad)))

    def _pitch_corregido(self):
        pitch_ff = -K_PITCH_FF * (abs(self.roll_active) / 10.0)
        return self.pitch_deg + pitch_ff

    def _pulsar_yaw(self, direccion):
        # Aplicamos el incremento al acumulador
        self.yaw_acum = max(-YAW_MAX, min(YAW_MAX, self.yaw_acum + direccion * YAW_STEP))
        # Setpoint absoluto = Orientación actual + desplazamiento
        self.yaw_active = self.psi_actual + self.yaw_acum
        # Roll coordinado
        self.roll_active = self.yaw_acum * BANK_POR_YAW
        limit = self._roll_max()
        self.roll_active = max(-limit, min(limit, self.roll_active))
        self.yaw_hold_until = time.time() + YAW_HOLD_TIME

    def _actualizar_resorte(self, now):
        if now >= self.yaw_hold_until:
            self.yaw_acum = 0.0
            self.roll_active = 0.0

    def _iniciar_transicion(self):
        self.A_cur = self._A_ef; self.T_cur = self._T_ef
        self.A_tgt = random.uniform(self.A_min, self.A_max)
        self.T_tgt = random.uniform(self.T_min, self.T_max)
        self.ramp_start = time.time()

    def _interpolar(self, now):
        if self.ramp_start is None: return self.A_cur, self.T_cur
        alpha = (now - self.ramp_start) / RAMP_DURATION
        if alpha >= 1.0:
            self.A_cur = self.A_tgt; self.T_cur = self.T_tgt
            self.ramp_start = None
            return self.A_cur, self.T_cur
        s = 0.5 * (1.0 - math.cos(math.pi * alpha))
        return (self.A_cur + s * (self.A_tgt - self.A_cur),
                self.T_cur + s * (self.T_tgt - self.T_cur))

    def publish_olas(self):
        now = time.time()
        if not self.ocean_enabled:
            self.pub_heave.publish(Float64(data=0.0))
            return
        if self.ramp_start is None and (now - self.last_interval) > INTERVAL:
            self._iniciar_transicion()
            self.last_interval = now
        A, T = self._interpolar(now)
        self._A_ef, self._T_ef = A, T
        omega = 2.0 * math.pi / T
        self.pub_heave.publish(Float64(data=float(A * math.cos(omega * (now - self.t0)))))

    def publish_all(self):
        self.pub_alt.publish(Float64(data=float(self._altura_corregida())))
        self.pub_pitch.publish(Float64(data=math.radians(self._pitch_corregido())))
        self.pub_yaw.publish(Float64(data=math.radians(self.yaw_active)))
        self.pub_roll.publish(Float64(data=math.radians(self.roll_active)))
        self.pub_turb.publish(Bool(data=self.turb_enabled))

    def shutdown_limpio(self):
        for pub in [self.pub_alt, self.pub_pitch, self.pub_yaw, self.pub_roll]:
            pub.publish(Float64(data=0.0))
        self.pub_turb.publish(Bool(data=False))

def main(stdscr):
    rclpy.init()
    node = VirtualRudder()
    stdscr.nodelay(True)
    stdscr.keypad(True)
    last_pub = time.time()

    try:
        while rclpy.ok():
            now = time.time()
            c = stdscr.getch()

            if c != -1:
                # CORRECCIÓN DE SIGNOS SEGÚN TEST:
                if   c == curses.KEY_RIGHT: node._pulsar_yaw(-1) # Invertido para que apunte a la derecha
                elif c == curses.KEY_LEFT:  node._pulsar_yaw(1)
                elif c == curses.KEY_UP:    node.pitch_deg -= node.step_deg # Invertido para corregir morro
                elif c == curses.KEY_DOWN:  node.pitch_deg += node.step_deg
                elif c in (ord('w'), ord('W')): node.alt_m += node.step_alt
                elif c in (ord('s'), ord('S')): node.alt_m -= node.step_alt
                elif c == ord(' '):
                    node.pitch_deg = 0.0; node.yaw_acum = 0.0
                    node.yaw_active = node.psi_actual; node.roll_active = 0.0
                elif c in (ord('t'), ord('T')): node.turb_enabled = not node.turb_enabled
                elif c in (ord('o'), ord('O')):
                    node.ocean_enabled = not node.ocean_enabled
                    node.t0 = now
                elif c in (ord('q'), ord('Q')): break

            node._actualizar_resorte(now)
            if now - last_pub >= 0.1:
                node.publish_all()
                last_pub = now
            
            # --- INTERFAZ GRAFICA COMPLETA ---
            stdscr.erase()
            stdscr.addstr(0, 0, '=== TIMON VIRTUAL v5 (RESORTE + CORRECCIÓN DE EJES) ===', curses.A_REVERSE)
            stdscr.addstr(1, 0, '[W/S] Altura  [UP/DOWN] Pitch  [◄/►] Yaw  [SPC] Reset  [Q] Salir')

            # Barra del resorte
            hold_restante = max(0.0, node.yaw_hold_until - now)
            bloques = int((hold_restante / YAW_HOLD_TIME) * 10)
            resorte_barra = '[' + '█' * bloques + ' ' * (10 - bloques) + ']'

            stdscr.addstr(4, 0, '--- VUELO ---', curses.A_BOLD)
            stdscr.addstr(5, 0, f'Altura SP  : {node.alt_m:.2f} m  (Real: {node.h_actual:.2f} m)')
            stdscr.addstr(6, 0, f'Pitch SP   : {node.pitch_deg:+.2f}° (Corr FF: {node._pitch_corregido():+.2f}°)')
            stdscr.addstr(7, 0, f'Yaw SP     : {node.yaw_active:+.1f}°  {resorte_barra}')
            stdscr.addstr(8, 0, f'Yaw Acum   : {node.yaw_acum:+.1f}° (Max: {YAW_MAX}°)')
            stdscr.addstr(9, 0, f'Roll SP    : {node.roll_active:+.2f}° (Lim: ±{node._roll_max():.1f}°)')

            # Perturbaciones
            turb_txt = 'ON' if node.turb_enabled else 'OFF'
            ocean_txt = f'ON (A:{node._A_ef:.2f}m T:{node._T_ef:.2f}s)' if node.ocean_enabled else 'OFF'
            stdscr.addstr(11, 0, '--- ESTADO ---', curses.A_BOLD)
            stdscr.addstr(12, 0, f'Turbulencia: {turb_txt}')
            stdscr.addstr(13, 0, f'Océano     : {ocean_txt}')
            
            stdscr.refresh()
            rclpy.spin_once(node, timeout_sec=0.01)

    finally:
        node.shutdown_limpio()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    curses.wrapper(main)
