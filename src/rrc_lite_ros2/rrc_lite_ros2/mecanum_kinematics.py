#!/usr/bin/env python3
"""
mecanum_kinematics.py
Cinemática directa e inversa para chassis Mecanum de 4 ruedas.

Disposición de motores (vista superior):
         +X (frontal)
    M1 ↗ ── ── ── ↖ M3
    |                |
+Y  |                | -Y
    |                |
    M2 ↘ ── ── ── ↙ M4
         -X (trasero)

Motor IDs en la RRC Lite:
    M1 = 1 (frontal izquierdo)
    M2 = 2 (trasero izquierdo)
    M3 = 3 (frontal derecho)
    M4 = 4 (trasero derecho)
"""

import math


class MecanumKinematics:
    """
    Cinemática de ruedas Mecanum para robot de 4 motores.

    Parámetros físicos (medir en el robot real):
        wheelbase    : distancia entre ejes frontal y trasero (metros)
        track_width  : distancia entre ruedas izquierda y derecha (metros)
        wheel_radius : radio de las ruedas Mecanum (metros)
    """

    def __init__(self, wheelbase: float = 0.148,
                 track_width: float = 0.140,
                 wheel_radius: float = 0.033):
        self.wheelbase   = wheelbase
        self.track_width = track_width
        self.wheel_radius = wheel_radius
        # L = (wheelbase + track_width) / 2  — factor cinemático
        self._L = (wheelbase + track_width) / 2.0

    def _ms_to_rps(self, speed_ms: float) -> float:
        """Convierte velocidad lineal (m/s) a RPS."""
        return speed_ms / (math.pi * 2.0 * self.wheel_radius)

    def cmd_vel_to_motor_rps(self, vx: float, vy: float,
                              wz: float) -> list[tuple[int, float]]:
        """
        Convierte velocidad deseada (vx, vy, wz) en RPS por motor.

        Args:
            vx : velocidad lineal delantera/trasera  (m/s)  +x = adelante
            vy : velocidad lateral                   (m/s)  +y = izquierda
            wz : velocidad angular                   (rad/s) +z = antihorario

        Returns:
            Lista de (motor_id, rps) para los 4 motores.
        """
        # Contribución de la rotación
        rot = wz * self._L

        # Velocidades de rueda en m/s
        v1 =  vx - vy - rot   # M1: frontal izquierdo
        v2 =  vx + vy - rot   # M2: trasero izquierdo
        v3 =  vx + vy + rot   # M3: frontal derecho
        v4 =  vx - vy + rot   # M4: trasero derecho

        # Convertir a RPS (signos: izquierda y derecha giran en sentido opuesto)
        rps1 = -self._ms_to_rps(v1)
        rps2 = -self._ms_to_rps(v2)
        rps3 =  self._ms_to_rps(v3)
        rps4 =  self._ms_to_rps(v4)

        return [(1, rps1), (2, rps2), (3, rps3), (4, rps4)]

    def motor_rps_to_cmd_vel(self, rps: list[float]) -> tuple[float, float, float]:
        """
        Cinemática directa: convierte RPS de motores en velocidad del chassis.

        Args:
            rps: [rps_M1, rps_M2, rps_M3, rps_M4]

        Returns:
            (vx, vy, wz) en (m/s, m/s, rad/s)
        """
        r = self.wheel_radius
        L = self._L

        # Compensar los signos de la disposición
        w1, w2, w3, w4 = -rps[0], -rps[1], rps[2], rps[3]

        vx = (w1 + w2 + w3 + w4) * r / 4.0
        vy = (-w1 + w2 + w3 - w4) * r / 4.0
        wz = (-w1 - w2 + w3 + w4) * r / (4.0 * L)

        return vx, vy, wz
