import numpy as np
from motor_model import Motor

class BoatController:
    def __init__(self, motor_params: Motor.Params, update_rate: float, L: float):
        self._m1 = Motor(motor_params, update_rate)
        self._m2 = Motor(motor_params, update_rate)
        self._m3 = Motor(motor_params, update_rate)
        self._m4 = Motor(motor_params, update_rate)
        
        self._dt = update_rate
        self._L = L
        
        self._v1 = 0.0
        self._v2 = 0.0
        self._v3 = 0.0
        self._v4 = 0.0
        
        # Generic factor for scaling speed
        self._kv = 1.0
        
        self._x = 0.0
        self._y = 0.0
        self._velocity = 0.0
        
        self._heading = 0.0 
        self._steering = 0.0
        
        self._depth = 1.0
        
        self._cntrl_dx = 0.0
        self._cntrl_dy = 0.0
        
    @property
    def depth(self):
        return self._depth
    
    @property
    def update_rate(self):
        return self._dt
        
    def run_kinematics(self):
        self._apply_commands()
        
        # self._heading = 0.0

        # self._heading = self._m1.position
        self._heading = np.arctan2(self._cntrl_dy, self._cntrl_dx)
        self._velocity = self._kv * (self._m2.velocity + self._m3.velocity)
        
        self._dx = self._velocity * np.cos(self._heading)
        self._dy = self._velocity * np.sin(self._heading)
        self._do = (self._velocity / self._L) * np.tan(self._heading)
        
        self._x += self._dx * self._dt
        self._y += self._dy * self._dt
        self._heading += self._do * self._dt
        
        # self._heading = 0.0
        # self._do = 0.0
        
    def transmit_commands(self, dx: float, dy: float):
        # IK, get direction and magnitude
        _do = np.arctan2(dy, dx)
        _v = (dx**2 + dy**2)**(0.5)
        self._cntrl_dx = dx
        self._cntrl_dy = dy
        
        # TODO: transmit over BLE.
        # For now, let's simply apply commands
        # We assume dx and dy to be normalized here (0, 1)
        # and our max voltage to be 24
        self._v1 = (_do / np.pi/2) * 24
        self._v2 = _v * 24
        self._v3 = _v * 24
        # self._apply_commands()
        
    @property
    def T(self):
        _R = self.R
        _t = self.t
        
        return np.array([
            [_R[0,0], _R[0,1], _R[0,2], _t[0]],
            [_R[1,0], _R[1,1], _R[1,2], _t[1]],
            [_R[2,0], _R[2,1], _R[2,2], _t[2]],
            [0, 0, 0, 1]
        ])
        
    @property
    def R(self):
        return np.array([
            [np.cos(self._heading), -np.sin(self._heading), 0.0],
            [np.sin(self._heading), np.cos(self._heading), 0.0],
            [0.0, 0.0, 1.0]
        ])
        
    @property
    def p(self):
        return np.array([self.x, self.y, 0.0]).T

    @property
    def T(self):
        _R = self.R
        _p = self.p
        
        return np.array([
            [_R[0,0], _R[0,1], _R[0,2], _p[0]],
            [_R[1,0], _R[1,1], _R[1,2], _p[1]],
            [_R[2,0], _R[2,1], _R[2,2], _p[2]],
            [0, 0, 0, 1]
        ])

    @property
    def dT(self):
        _dR = self.dR
        _dp = self.dp
        
        return np.array([
            [_dR[0,0], _dR[0,1], _dR[0,2], _dp[0]],
            [_dR[1,0], _dR[1,1], _dR[1,2], _dp[1]],
            [_dR[2,0], _dR[2,1], _dR[2,2], _dp[2]],
            [0, 0, 0, 1]
        ])
        
    @property
    def R(self):
        return np.array([
            [np.cos(-self._heading), -np.sin(-self._heading), 0.0],
            [np.sin(-self._heading), np.cos(-self._heading), 0.0],
            [0.0, 0.0, 1.0]
        ])

    @property
    def dR(self):
        return np.array([
            [np.cos(-self._do), -np.sin(-self._do), 0.0],
            [np.sin(-self._do), np.cos(-self._do), 0.0],
            [0.0, 0.0, 1.0]
        ])
        
    @property
    def dp(self):
        return np.array([self.dx, self.dy, 0.0]).T
        
    @property
    def heading(self):
        return np.degrees(self._heading)
    
    @property
    def dh(self):
        return np.degrees(self._do)
    
    @property
    def x(self):
        return self._x
    
    @property
    def dx(self):
        return self._dx
    
    @property
    def y(self):
        return self._y
    
    @property
    def dy(self):
        return self._dy
        
    def _apply_commands(self):
        self._m1.sim(self._v1)
        self._m2.sim(self._v2)
        self._m3.sim(self._v3)
        self._m4.sim(self._v4)