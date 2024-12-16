import numpy as np
import queue

from motor_model import Motor
from ble_listener import MotorboatBLEListener, BoatState, MotorState, MotorType

class BoatController:
    def __init__(self, motor_params: Motor.Params, update_rate: float, ble_comms: MotorboatBLEListener):
        self._m1 = Motor(motor_params, update_rate)
        self._m2 = Motor(motor_params, update_rate)
        self._m3 = Motor(motor_params, update_rate)
        self._m4 = Motor(motor_params, update_rate)
        
        self._ble_comms = ble_comms
        
        self._dt = update_rate
        
        self._v_ref = 24.0
        self._v1 = 0.0
        self._v2 = 0.0
        self._v3 = 0.0
        self._v4 = 0.0
        
        # Generic factor for scaling speed
        self._kv = 1.0
        
        self._state = BoatState.BOAT_IDLE
        self._motor_states = [MotorState(False, False, False) for _ in range(5)]
        self._ts = 0
        self._roll = 0.0
        self._pitch = 0.0
        self._x = 0.0
        self._y = 0.0
        self._velocity = 0.0
        self._delta = 0.0
        self._temperature = 0
        
        self._heading = 0.0 
        self._steering = 0.0
        
        self._depth = 0
        self._c = 0
                 
    @property
    def state(self):
        return self._state
                        
    @property
    def temperature(self):
        return self._temperature
                        
    @property
    def timestamp(self):
        return self._ts                        
                        
    @property
    def depth(self):
        return self._depth
    
    @property
    def update_rate(self):
        return self._dt
    
    @property
    def roll(self):
        return self._roll
    
    @property
    def pitch(self):
        return self._pitch
        
    def run_kinematics(self):
        self._apply_commands()
    
        # Motor space -> "joint" space
        self._delta    = self._m1.velocity
        self._velocity = self._kv * (self._m2.velocity + self._m3.velocity)

        # Compute kinematics
        self._dx = (self._velocity * np.cos(np.pi/2 + self._heading)) * self._dt
        self._dy = (self._velocity * np.sin(np.pi/2 + self._heading)) * self._dt
        self._do = self._delta * self._dt

        # Discrete differentiation -> map velocities to position/orientation
        self._x += self._dx
        self._y += self._dy
        self._heading += self._do
        
    def transmit_commands(self, dx: float, dy: float, dz: int):
        # IK, get direction and magnitude
        _do = np.arctan2(dy, dx)
        _v = dz
        
        if abs(dx) < 1e-5: 
            _do = 0.0
        else:
            _do = np.sign(dy) * (_do - (np.sign(dy) * np.pi/2))

        self._ble_comms.send_boat_motion_command(_do, _v)
        
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
            [np.cos(self._heading), -np.sin(self._heading), 0.0],
            [np.sin(self._heading), np.cos(self._heading), 0.0],
            [0.0, 0.0, 1.0]
        ])

    @property
    def dR(self):
        return np.array([
            [np.cos(self._do), -np.sin(self._do), 0.0],
            [np.sin(self._do), np.cos(self._do), 0.0],
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
    
    @property
    def delta(self):
        return np.degrees(self._delta)
    
    @property
    def velocity(self):
        return self._velocity
    
    @property
    def thurst_motors(self):
        # hacky
        return [self._m2, self._m3, self._m4]
    
    def _apply_commands(self):
        try:
            packet = self._ble_comms.pop_command()
        except queue.Empty:
            return
        
        self._c += 1
        if self._c % 50 == 0:
            print(packet)   
        
        self._ts = packet.timestamp
        
        # Get roll/pitch, data is already in degrees
        self._roll = packet.roll
        self._pitch = packet.pitch
        
        # Get depth
        self._depth = packet.depth
        
        # Get temperature
        self._temperature = packet.temperature
        
        # Get the boat state
        self._state = packet.boat_state
        
        thurst_motor_index = 0
        for state in packet.motor_states:
            # Simualte the motors
            if state.motor_state.motor_type == MotorType.STEERING and state.motor_state.is_alive:
                motor = self._m1
            elif state.motor_state.motor_type == MotorType.THRUST and state.motor_state.is_alive:
                motor = self.thurst_motors[thurst_motor_index]
                thurst_motor_index += 1
            else:
                continue
                
            motor.sim(state.direction * self._v_ref * state.duty_cycle / 100)