from dataclasses import dataclass
import scipy.signal as signal
import numpy as np

class Motor:
    @dataclass(frozen = True)
    class Params:
        R: float
        L: float
        J: float
        b: float
        K: float
        
    def __init__(self, params: Params, dt: float):
        """https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemModeling"""
        self._params = params
        self._dt = dt

        # State-space representation
        _A = np.array([[-self._params.b / self._params.J, self._params.K / self._params.J],
                       [-self._params.K / self._params.L, -self._params.R / self._params.L]])
        _B = np.array([[0], [1 / self._params.L]])
        _C = np.array([[1, 0]])
        _D = np.array([[0]])
        
        # Discrete system, taken from continuous state-space representation
        self._sys = signal.cont2discrete(
            (_A, _B, _C, _D), dt = self._dt
        )
                
        # Initialize output variables
        self._x0 = [0.0, 0.0]
        self._t = 0.0
        self._w = 0.0
        self._theta = 0.0
        
        self._previous_voltage = 0.0
        
    def sim(self, V: float):
        _t, _w, _x0 = signal.dlsim(
            self._sys, 
            [self._previous_voltage, V], 
            x0 = self._x0
        )
        
        self._t += np.squeeze(_t)[-1]
        self._w = np.squeeze(_w)[-1]
        self._x0 = _x0[1:, :]
        self._theta += self._w * self._dt
        self._previous_voltage = V
    
    @property
    def position(self):
        return self._theta
    
    @property
    def velocity(self):
        return self._w
    
if __name__ == "__main__":
    motor = Motor(
        params = Motor.Params(R = 1, L = 0.5, J = 0.01, b = 0.1, K = 0.01), 
        dt = 0.01
    )
    
    for _ in range(10000):
        motor.sim(1)