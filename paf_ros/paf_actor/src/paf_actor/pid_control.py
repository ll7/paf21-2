from collections import deque
from typing import Deque


class PIDLongitudinalController(object):  # pylint: disable=too-few-public-methods
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    """

    def __init__(self, K_P: float = 1.0, K_D: float = 0.0, K_I: float = 0.0):
        """
        Constructor

        Args:
            K_P (float, optional): Proportional term. Defaults to 1.0.
            K_D (float, optional): Differential term. Defaults to 0.0.
            K_I (float, optional): Integral term. Defaults to 0.0.
        """
        self._K_P: float = K_P
        self._K_D: float = K_D
        self._K_I: float = K_I
        self._e_buffer: Deque = deque(maxlen=30)

    def run_step(self, target_speed: float, current_speed: float, dt: float) -> float:
        """
        Estimate the throttle of the vehicle based on the PID equations

        Args:
            target_speed (float): Target speed in Km/h
            current_speed (float): Current speed of the vehicle in Km/h
            dt (float): time diffrence

        Returns:
            float: Throttle control in the range [0, 1]
        """
        _e = target_speed - current_speed
        self._e_buffer.append(_e)

        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / dt
            _ie = sum(self._e_buffer) * dt
        else:
            _de = 0.0
            _ie = 0.0

        return (self._K_P * _e) + (self._K_D * _de / dt) + (self._K_I * _ie * dt)
