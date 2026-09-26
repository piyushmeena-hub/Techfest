"""
sim/weather.py: Industrial Atmospheric Physics & Dryden Stochastic Wind Turbulence Engine.

Implements:
1. MIL-F-8785C / FAA standard low-altitude Dryden wind turbulence model.
2. Boundary layer altitude wind shear power law.
3. Building wake aerodynamic downdrafts on the leeward side of obstacle structures.
4. Deterministic and stochastic gust injection for swarm aerodynamic stress testing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, List, Optional, Sequence, Tuple
import numpy as np


@dataclass
class WindConfig:
    """Atmospheric environmental parameters."""
    mean_speed_mps: float = 3.5            # Mean horizontal wind speed (m/s) at reference altitude
    direction_deg: float = 45.0           # Meteorological wind direction (degrees from North)
    ref_altitude_m: float = 10.0          # Reference altitude for mean speed
    shear_exponent: float = 0.143         # Atmospheric boundary layer power-law exponent (open terrain)
    turbulence_intensity: str = "MODERATE"# 'LIGHT' | 'MODERATE' | 'SEVERE'
    gust_probability: float = 0.02        # Probability of transient microburst gust per second
    gust_magnitude_mps: float = 6.0       # Peak gust velocity increment (m/s)


class DrydenTurbulenceModel:
    """
    MIL-F-8785C Low-Altitude Dryden Spectral Shaping Turbulence Filter.
    
    Generates realistic 3D stochastic velocity fluctuations (u_g, v_g, w_g)
    as a function of flight altitude, airspeed, and atmospheric stability.
    """

    def __init__(self, config: Optional[WindConfig] = None) -> None:
        self.config = config if config is not None else WindConfig()
        
        # State registers for continuous-time shaping filters (Euler discretized)
        self._state_u: float = 0.0
        self._state_v1: float = 0.0
        self._state_v2: float = 0.0
        self._state_w1: float = 0.0
        self._state_w2: float = 0.0

        # Active microburst gust state
        self._gust_active: bool = False
        self._gust_timer: float = 0.0
        self._gust_duration: float = 3.0
        self._gust_vector: np.ndarray = np.zeros(3, dtype=np.float64)

        # Intensity scale factors
        intensity_map = {
            "LIGHT": 0.5,
            "MODERATE": 1.0,
            "SEVERE": 2.2,
        }
        self._intensity_scale = intensity_map.get(self.config.turbulence_intensity.upper(), 1.0)

    def get_mean_wind(self, altitude_m: float) -> np.ndarray:
        """
        Calculate horizontal mean wind vector at altitude z using power-law boundary layer shear.
        """
        z = max(0.5, float(altitude_m))
        z_ref = max(1.0, self.config.ref_altitude_m)
        alpha = self.config.shear_exponent
        
        # Power-law wind speed scaling
        speed = self.config.mean_speed_mps * ((z / z_ref) ** alpha)
        speed = min(speed, 25.0)  # Hard ceiling for low-altitude sanity

        # Convert meteorological direction (coming FROM angle) to Cartesian velocity vector
        theta = math.radians(self.config.direction_deg)
        # Wind blowing TOWARDS (cos(theta), sin(theta))
        vx = -speed * math.sin(theta)
        vy = -speed * math.cos(theta)
        return np.array([vx, vy, 0.0], dtype=np.float64)

    def step_turbulence(self, altitude_m: float, dt: float) -> np.ndarray:
        """
        Advance Dryden stochastic state filters and return 3D gust velocity vector (u_g, v_g, w_g).
        """
        h = max(2.0, min(float(altitude_m), 300.0))  # Valid for h in [2m, 300m]
        
        # MIL-F-8785C scale lengths (meters)
        L_w = h
        L_u = h / ((0.177 + 0.000823 * h) ** 1.2)
        L_v = L_u

        # Base turbulence intensity at 6m (20ft)
        sigma_w0 = 0.15 * self.config.mean_speed_mps * self._intensity_scale
        sigma_w = max(0.2, min(sigma_w0, 8.0))
        sigma_u = sigma_w / ((0.177 + 0.000823 * h) ** 0.4)
        sigma_v = sigma_u

        # Equivalent forward flight speed for spatial-to-temporal transformation
        V = max(5.0, self.config.mean_speed_mps)

        # White noise drive
        w_u = float(np.random.normal(0.0, 1.0))
        w_v = float(np.random.normal(0.0, 1.0))
        w_w = float(np.random.normal(0.0, 1.0))

        # Discretized first-order filter for u_g: tau = L_u / V
        tau_u = max(0.1, L_u / V)
        alpha_u = math.exp(-dt / tau_u)
        gain_u = sigma_u * math.sqrt(2.0 * (1.0 - alpha_u ** 2))
        self._state_u = alpha_u * self._state_u + gain_u * w_u

        # Discretized second-order filter for v_g: tau = L_v / V
        tau_v = max(0.1, L_v / V)
        alpha_v = math.exp(-dt / tau_v)
        gain_v = sigma_v * math.sqrt(2.0 * (1.0 - alpha_v ** 2))
        self._state_v1 = alpha_v * self._state_v1 + gain_v * w_v

        # Discretized second-order filter for w_g: tau = L_w / V
        tau_w = max(0.1, L_w / V)
        alpha_w = math.exp(-dt / tau_w)
        gain_w = sigma_w * math.sqrt(2.0 * (1.0 - alpha_w ** 2))
        self._state_w1 = alpha_w * self._state_w1 + gain_w * w_w

        return np.array([self._state_u, self._state_v1, self._state_w1], dtype=np.float64)

    def _update_gusts(self, dt: float) -> np.ndarray:
        """Process intermittent discrete microbursts / thermal updrafts."""
        if not self._gust_active:
            # Stochastic trigger check
            if np.random.uniform(0.0, 1.0) < (self.config.gust_probability * dt):
                self._gust_active = True
                self._gust_timer = 0.0
                self._gust_duration = float(np.random.uniform(2.0, 5.0))
                # Random 3D gust vector with vertical component
                phi = np.random.uniform(0, 2 * math.pi)
                mag = self.config.gust_magnitude_mps * self._intensity_scale
                self._gust_vector = np.array([
                    mag * math.cos(phi),
                    mag * math.sin(phi),
                    np.random.uniform(-mag * 0.4, mag * 0.6),
                ], dtype=np.float64)
        
        if self._gust_active:
            self._gust_timer += dt
            if self._gust_timer >= self._gust_duration:
                self._gust_active = False
                return np.zeros(3, dtype=np.float64)
            # 1 - cos half-sine envelope (FAA standard gust shape)
            fraction = self._gust_timer / self._gust_duration
            envelope = 0.5 * (1.0 - math.cos(2.0 * math.pi * fraction))
            return self._gust_vector * envelope

        return np.zeros(3, dtype=np.float64)

    def compute_building_wake(
        self,
        position: np.ndarray,
        mean_wind: np.ndarray,
        obstacles: Sequence[Any],
    ) -> np.ndarray:
        """
        Calculates leeward aerodynamic downdraft and recirculation vortex behind buildings.
        """
        v_wake = np.zeros(3, dtype=np.float64)
        wind_speed = float(np.linalg.norm(mean_wind[:2]))
        if wind_speed < 1.0 or not obstacles:
            return v_wake

        wind_dir = mean_wind[:2] / wind_speed

        p = np.asarray(position, dtype=np.float64)
        for obs in obstacles:
            min_p = getattr(obs, "min_pt", getattr(obs, "min_bound", None))
            max_p = getattr(obs, "max_pt", getattr(obs, "max_bound", None))
            if min_p is None or max_p is None:
                continue

            center = 0.5 * (min_p + max_p)
            height = float(max_p[2] - min_p[2])
            width_x = float(max_p[0] - min_p[0])
            width_y = float(max_p[1] - min_p[1])
            char_width = max(width_x, width_y)

            # Check if drone is in downwind wake zone (within 2.5 building heights downwind)
            diff = p[:2] - center[:2]
            downwind_dist = float(np.dot(diff, wind_dir))
            crosswind_dist = float(np.abs(diff[0] * wind_dir[1] - diff[1] * wind_dir[0]))

            if 0.0 < downwind_dist < (2.5 * height) and crosswind_dist < (char_width * 0.8):
                if p[2] <= (height * 1.2):
                    # In the wake cavity! Downward suction and horizontal deceleration
                    decay = (1.0 - downwind_dist / (2.5 * height))
                    w_downdraft = -0.4 * wind_speed * decay * (1.0 - crosswind_dist / (char_width * 0.8))
                    u_deficit = -0.6 * wind_speed * decay
                    v_wake[0] += wind_dir[0] * u_deficit
                    v_wake[1] += wind_dir[1] * u_deficit
                    v_wake[2] += w_downdraft

        return v_wake

    def get_wind_at(
        self,
        position: Sequence[float],
        dt: float = 0.05,
        obstacles: Optional[Sequence[Any]] = None,
    ) -> np.ndarray:
        """
        Calculates composite instantaneous wind vector at given 3D position:
        V_net = V_mean(z) + V_dryden(z) + V_gust + V_wake(buildings)
        """
        p = np.asarray(position, dtype=np.float64)
        v_mean = self.get_mean_wind(p[2])
        v_dryden = self.step_turbulence(p[2], dt)
        v_gust = self._update_gusts(dt)
        v_wake = self.compute_building_wake(p, v_mean, obstacles or [])

        return v_mean + v_dryden + v_gust + v_wake

    def sample(
        self,
        position: Sequence[float],
        velocity: Optional[Sequence[float]] = None,
        dt: float = 0.05,
        obstacles: Optional[Sequence[Any]] = None,
    ) -> np.ndarray:
        """Convenience alias for get_wind_at, accepting optional velocity for API symmetry."""
        return self.get_wind_at(position, dt=dt, obstacles=obstacles)
