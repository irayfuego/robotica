#!/usr/bin/env python3
"""
mecanum_kinematics.py
Cinemática directa e inversa para chassis Mecanum de 4 ruedas.

Disposición física REAL (vista superior):
         +X (frontal)
    M1 ┌─────────┐ M2
       │         │
 +Y    │         │   -Y
       │         │
    M3 └─────────┘ M4
         -X (trasero)

Motor IDs en la RRC Lite (mapeo físico confirmado 2026-06-26):
    M1 = 1  frontal IZQUIERDO  (FL)
    M2 = 2  frontal DERECHO    (FR)
    M3 = 3  trasero IZQUIERDO  (RL)
    M4 = 4  trasero DERECHO    (RR)

Sentido de giro confirmado (rps POSITIVO en set_motor_speed):
    M1 -> atrás     M2 -> adelante
    M3 -> atrás     M4 -> adelante
Es decir, las ruedas izquierdas (M1, M3) están montadas en espejo y giran
invertidas respecto a las derechas. Para que una rueda ruede HACIA ADELANTE:
    izquierdas (M1, M3): rps NEGATIVO
    derechas   (M2, M4): rps POSITIVO
Validado: "todas adelante" = [[1,-0.5],[2,0.5],[3,-0.5],[4,0.5]].

MONTAJE GIRADO 180° (2026-06-27): el robot se monto con la trasera como frente
(FL=M4, FR=M3, RL=M2, RR=M1). Se corrige con body_reversed=True, que invierte
vx y vy (el giro wz NO cambia con una rotacion de 180°). Demostrado que esto
EQUIVALE al remapeo de motores (simetria del mecanum).

NOTA: los signos de vy (lateral) y wz (giro) siguen la convención Mecanum
estándar (rodillos en X). Si al probar el lateral o el giro salen invertidos,
basta cambiar el signo de la componente correspondiente.
"""

import math


class MecanumKinematics:
    """
    Cinemática de ruedas Mecanum para robot de 4 motores.

    Parámetros físicos (medir en el robot real):
        wheelbase    : distancia entre ejes frontal y trasero (metros)
        track_width  : distancia entre ruedas izquierda y derecha (metros)
        wheel_radius : radio de las ruedas Mecanum (metros)
        rps_calib    : factor de calibración de velocidad. El firmware de la
                       RRC está ajustado para el motor Hiwonder de fábrica; con
                       el NULLLAB (12 PPR, 1:90) la velocidad real sale a ~22%
                       de lo comandado. Medido: cmd 0.45 m/s -> 0.10 real, asi
                       que rps_calib = 1/0.22 = 4.5 (multiplica el rps de salida).
        body_reversed: True si el robot esta MONTADO girado 180° (la trasera
                       quedo como frente). Invierte vx y vy para que los comandos
                       coincidan con el frente real (el giro wz no cambia).
    """

    def __init__(self, wheelbase: float = 0.148,
                 track_width: float = 0.140,
                 wheel_radius: float = 0.033,
                 rps_calib: float = 1.0,
                 body_reversed: bool = False):
        self.wheelbase    = wheelbase
        self.track_width  = track_width
        self.wheel_radius = wheel_radius
        self.rps_calib    = rps_calib
        self.body_reversed = body_reversed
        # L = (wheelbase + track_width) / 2  — brazo de palanca para la rotación
        self._L = (wheelbase + track_width) / 2.0
        # circunferencia efectiva: v_lineal (m/s) = rps * _circ
        self._circ = 2.0 * math.pi * self.wheel_radius

    def _ms_to_rps(self, speed_ms: float) -> float:
        """Velocidad lineal de rueda (m/s) -> revoluciones por segundo."""
        return speed_ms / self._circ

    def _rps_to_ms(self, rps: float) -> float:
        """Revoluciones por segundo -> velocidad lineal de rueda (m/s)."""
        return rps * self._circ

    def cmd_vel_to_motor_rps(self, vx: float, vy: float,
                              wz: float) -> list[tuple[int, float]]:
        """
        Cinemática inversa: velocidad deseada del chassis -> RPS por motor.

        Args:
            vx : velocidad adelante/atrás  (m/s)   +x = adelante
            vy : velocidad lateral         (m/s)   +y = izquierda
            wz : velocidad angular         (rad/s) +z = antihorario

        Returns:
            Lista [(motor_id, rps), ...] lista para set_motor_speed.
        """
        # Robot montado girado 180°: el chasis ve vx/vy invertidos (wz igual).
        if self.body_reversed:
            vx = -vx
            vy = -vy

        rot = wz * self._L

        # Velocidad lineal de rodadura de cada rueda (m/s), Mecanum estándar
        v_fl = vx - vy - rot   # M1 frontal izquierda
        v_fr = vx + vy + rot   # M2 frontal derecha
        v_rl = vx + vy - rot   # M3 trasera izquierda
        v_rr = vx - vy + rot   # M4 trasera derecha

        # A RPS de motor, aplicando el signo físico de montaje
        # (izquierdas invertidas) y el factor de calibración.
        k = self.rps_calib
        rps_m1 = -self._ms_to_rps(v_fl) * k   # FL izquierda -> invertida
        rps_m2 =  self._ms_to_rps(v_fr) * k   # FR derecha   -> directa
        rps_m3 = -self._ms_to_rps(v_rl) * k   # RL izquierda -> invertida
        rps_m4 =  self._ms_to_rps(v_rr) * k   # RR derecha   -> directa

        return [(1, rps_m1), (2, rps_m2), (3, rps_m3), (4, rps_m4)]

    def motor_rps_to_cmd_vel(self, rps: list[float]) -> tuple[float, float, float]:
        """
        Cinemática directa: RPS de los 4 motores -> velocidad del chassis.

        Args:
            rps: [rps_M1, rps_M2, rps_M3, rps_M4]  (tal cual los lee la placa)

        Returns:
            (vx, vy, wz) en (m/s, m/s, rad/s)
        """
        L = self._L

        # Deshacer el factor de calibración y el signo físico de montaje
        # -> velocidad lineal de cada rueda.
        k = self.rps_calib if self.rps_calib else 1.0
        v_fl = self._rps_to_ms(-rps[0] / k)   # M1 FL izquierda (invertida)
        v_fr = self._rps_to_ms( rps[1] / k)   # M2 FR derecha
        v_rl = self._rps_to_ms(-rps[2] / k)   # M3 RL izquierda (invertida)
        v_rr = self._rps_to_ms( rps[3] / k)   # M4 RR derecha

        vx = (v_fl + v_fr + v_rl + v_rr) / 4.0
        vy = (-v_fl + v_fr + v_rl - v_rr) / 4.0
        wz = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * L)

        # Deshacer la inversion del montaje girado 180° (coherente con cmd_vel).
        if self.body_reversed:
            vx = -vx
            vy = -vy

        return vx, vy, wz
