import numpy as np
from scipy.integrate import odeint
from scipy.signal import lti, step
 
class PIDController:
    """Discrete PID controller with anti-windup."""
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0,
                 setpoint=1.0, dt=0.01, output_limits=(-100, 100)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.dt = dt
        self.limits = output_limits
        self.integral = 0.0
        self.prev_error = 0.0
    
    def update(self, measured_value):
        error = self.setpoint - measured_value
        # Proportional
        P = self.Kp * error
        # Integral with anti-windup
        self.integral += error * self.dt
        I = self.Ki * self.integral
        # Derivative (on error)
        derivative = (error - self.prev_error) / self.dt
        D = self.Kd * derivative
        self.prev_error = error
        # Total output with clamping
        output = P + I + D
        output = max(self.limits[0], min(self.limits[1], output))
        # Anti-windup: clamp integral if output saturated
        if output >= self.limits[1] or output <= self.limits[0]:
            self.integral -= error * self.dt
        return output, P, I, D, error
 
def first_order_plant(y, t, u, K, tau):
    """First-order system: tau * dy/dt + y = K * u"""
    dydt = (K * u - y) / tau
    return dydt
 
def second_order_plant(state, t, u, K, wn, zeta):
    """Second-order system: y'' + 2*zeta*wn*y' + wn^2*y = K*wn^2*u"""
    y, ydot = state
    yddot = K * wn**2 * u - 2 * zeta * wn * ydot - wn**2 * y
    return [ydot, yddot]
 
def simulate_pid(Kp, Ki, Kd, plant_type="first_order",
                  plant_gain=1.0, plant_tau=1.0,
                  plant_wn=1.0, plant_zeta=0.5,
                  setpoint=1.0, t_end=20.0, dt=0.01,
                  disturbance_time=None, disturbance_mag=0.0):
    """Run closed-loop PID simulation."""
    t = np.arange(0, t_end, dt)
    n = len(t)
    
    pid = PIDController(Kp, Ki, Kd, setpoint, dt)
    
    y_out = np.zeros(n)
    u_out = np.zeros(n)
    e_out = np.zeros(n)
    sp_out = np.full(n, setpoint)
    p_out = np.zeros(n)
    i_out = np.zeros(n)
    d_out = np.zeros(n)
    
    y = 0.0
    state = [0.0, 0.0]  # for 2nd order
    
    for i in range(1, n):
        # PID output
        u, P, I, D, error = pid.update(y)
        
        # Add disturbance
        dist = 0.0
        if disturbance_time and t[i] >= disturbance_time:
            dist = disturbance_mag
        
        # Plant simulation (one step)
        if plant_type == "first_order":
            y_next = odeint(first_order_plant, y,
                            [t[i-1], t[i]],
                            args=(u + dist, plant_gain, plant_tau))
            y = y_next[-1]
        else:
            state_next = odeint(second_order_plant, state,
                                 [t[i-1], t[i]],
                                 args=(u + dist, plant_gain,
                                       plant_wn, plant_zeta))
            state = state_next[-1]
            y = state[0]
        
        y_out[i] = y
        u_out[i] = u
        e_out[i] = error
        p_out[i] = P
        i_out[i] = I
        d_out[i] = D
    
    # Performance metrics
    # Rise time (10% to 90% of setpoint)
    try:
        t_10 = t[np.where(y_out >= 0.1 * setpoint)[0][0]]
        t_90 = t[np.where(y_out >= 0.9 * setpoint)[0][0]]
        rise_time = t_90 - t_10
    except IndexError:
        rise_time = float('inf')
    
    # Overshoot
    overshoot = max(0, (max(y_out) - setpoint) / setpoint * 100)
    
    # Settling time (within 2% of setpoint)
    settled = np.where(np.abs(y_out - setpoint) <= 0.02 * setpoint)[0]
    if len(settled) > 0:
        # Find last time it leaves the band
        outside = np.where(np.abs(y_out - setpoint) > 0.02 * setpoint)[0]
        if len(outside) > 0:
            settling_time = t[outside[-1]]
        else:
            settling_time = 0.0
    else:
        settling_time = t_end
    
    # Steady-state error
    ss_error = abs(setpoint - np.mean(y_out[-100:]))
    
    # IAE (Integral of Absolute Error)
    iae = np.sum(np.abs(e_out)) * dt
    
    return {
        "t": t, "y": y_out, "u": u_out, "e": e_out,
        "sp": sp_out, "P": p_out, "I": i_out, "D": d_out,
        "rise_time": round(rise_time, 3),
        "overshoot_pct": round(overshoot, 2),
        "settling_time": round(settling_time, 3),
        "ss_error": round(ss_error, 5),
        "iae": round(iae, 3),
    }
 
def ziegler_nichols_tuning(Ku, Tu):
    """Ziegler-Nichols ultimate gain method."""
    return {
        "P-only":  {"Kp": round(0.5 * Ku, 3), "Ki": 0, "Kd": 0},
        "PI":      {"Kp": round(0.45 * Ku, 3),
                    "Ki": round(0.45 * Ku / (Tu/1.2), 3), "Kd": 0},
        "PID":     {"Kp": round(0.6 * Ku, 3),
                    "Ki": round(0.6 * Ku / (Tu/2), 3),
                    "Kd": round(0.6 * Ku * Tu/8, 3)},
    }
 
if __name__ == "__main__":
    r = simulate_pid(Kp=2.0, Ki=1.0, Kd=0.5,
                      plant_type="second_order",
                      plant_zeta=0.3, t_end=15)
    print(f"Rise time: {r['rise_time']}s")
    print(f"Overshoot: {r['overshoot_pct']}%")
    print(f"Settling time: {r['settling_time']}s")
    print(f"SS error: {r['ss_error']}")
    print(f"IAE: {r['iae']}")
    zn = ziegler_nichols_tuning(Ku=4.0, Tu=2.0)
    print(f"Z-N tuning: {zn}")
