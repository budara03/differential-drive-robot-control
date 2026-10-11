import math
from dataclasses import dataclass, field

def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))

def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))

@dataclass
class RobotParams:
    wheel_base: float = 50.0
    max_wheel_speed: float = 200.0
    motor_tau: float = 0.35
    wheel_mismatch: float = 0.04

class DifferentialDriveRobot:

    def __init__(self, x, y, theta, params: RobotParams = None):
        self.p = params if params is not None else RobotParams()
        self.x, self.y, self.theta = float(x), float(y), float(theta)
        self.v_left = 0.0
        self.v_right = 0.0

    @staticmethod
    def inverse_kinematics(v, omega, wheel_base):
        """(v, omega) -> (v_left, v_right)."""
        v_left = v - omega * wheel_base / 2.0
        v_right = v + omega * wheel_base / 2.0
        return v_left, v_right

    @staticmethod
    def forward_kinematics(v_left, v_right, wheel_base):
        """(v_left, v_right) -> (v, omega)."""
        v = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / wheel_base
        return v, omega

    def step(self, v_cmd, omega_cmd, dt):
        p = self.p

        vl_cmd, vr_cmd = self.inverse_kinematics(v_cmd, omega_cmd, p.wheel_base)
        vl_cmd = clamp(vl_cmd, -p.max_wheel_speed, p.max_wheel_speed)
        vr_cmd = clamp(vr_cmd, -p.max_wheel_speed, p.max_wheel_speed)

        alpha = 1.0 - math.exp(-dt / p.motor_tau)
        self.v_left += alpha * (vl_cmd - self.v_left)
        self.v_right += alpha * ((vr_cmd * (1.0 - p.wheel_mismatch)) - self.v_right)

        v, omega = self.forward_kinematics(self.v_left, self.v_right, p.wheel_base)

        theta_mid = self.theta + 0.5 * omega * dt
        self.x += v * math.cos(theta_mid) * dt
        self.y += v * math.sin(theta_mid) * dt
        self.theta = wrap_angle(self.theta + omega * dt)
        return v, omega

    def stop(self):
        self.v_left = 0.0
        self.v_right = 0.0

    @property
    def speed(self):
        return (self.v_left + self.v_right) / 2.0

class PDistance:

    name = "P"

    def __init__(self, kp=3.0, v_max=180.0):
        self.kp, self.v_max = kp, v_max

    def reset(self, distance_error=None):
        pass

    def update(self, distance_error, dt):
        return clamp(self.kp * distance_error, 0.0, self.v_max)

class PDDistance:

    name = "PD"

    def __init__(self, kp=3.0, kd=1.0, v_max=180.0, filter_alpha=0.3):
        self.kp, self.kd, self.v_max = kp, kd, v_max
        self.alpha = filter_alpha
        self.reset()

    def reset(self, distance_error=None):
        self.prev_error = distance_error
        self.d_filtered = 0.0

    def update(self, distance_error, dt):
        if self.prev_error is None:
            self.prev_error = distance_error
        raw = (distance_error - self.prev_error) / dt
        self.prev_error = distance_error
        self.d_filtered += self.alpha * (raw - self.d_filtered)
        v = self.kp * distance_error + self.kd * self.d_filtered
        return clamp(v, 0.0, self.v_max)

class PHeading:
    name = "P"

    def __init__(self, kp=2.0, omega_max=4.0):
        self.kp, self.omega_max = kp, omega_max

    def reset(self, heading_error=None):
        pass

    def update(self, heading_error, dt):
        return clamp(self.kp * heading_error, -self.omega_max, self.omega_max)

class PIDHeading:

    name = "PID"

    def __init__(self, kp=2.0, ki=0.8, kd=0.25, i_limit=1.0, omega_max=4.0,
                 i_zone=math.radians(25)):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit, self.omega_max, self.i_zone = i_limit, omega_max, i_zone
        self.reset()

    def reset(self, heading_error=None):
        self.integral = 0.0
        self.prev_error = heading_error

    def update(self, heading_error, dt):
        if self.prev_error is None:
            self.prev_error = heading_error
        if abs(heading_error) < self.i_zone:
            self.integral = clamp(self.integral + heading_error * dt,
                                  -self.i_limit, self.i_limit)
        else:
            self.integral = 0.0
        derivative = wrap_angle(heading_error - self.prev_error) / dt
        self.prev_error = heading_error
        omega = (self.kp * heading_error
                 + self.ki * self.integral
                 + self.kd * derivative)
        return clamp(omega, -self.omega_max, self.omega_max)

CONTROLLER_MODES = {
    "P":   ("P distance + P heading",    PDistance,  PHeading),
    "PD":  ("PD distance + P heading",   PDDistance, PHeading),
    "PID": ("PD distance + PID heading", PDDistance, PIDHeading),
}

@dataclass
class ControllerSet:
    mode: str = "PID"
    distance: object = field(init=False)
    heading: object = field(init=False)

    def __post_init__(self):
        _, dist_cls, head_cls = CONTROLLER_MODES[self.mode]
        self.distance = dist_cls()
        self.heading = head_cls()

    @property
    def label(self):
        return CONTROLLER_MODES[self.mode][0]

    def reset(self, distance_error=None, heading_error=None):
        self.distance.reset(distance_error)
        self.heading.reset(heading_error)

    def compute(self, robot, target, dt):
        dx, dy = target[0] - robot.x, target[1] - robot.y
        distance_error = math.hypot(dx, dy)
        desired_theta = math.atan2(dy, dx)
        heading_error = wrap_angle(desired_theta - robot.theta)

        v_cmd = self.distance.update(distance_error, dt)
        v_cmd *= max(0.0, math.cos(heading_error))
        omega_cmd = self.heading.update(heading_error, dt)
        return v_cmd, omega_cmd, distance_error, heading_error

STOP_DISTANCE = 5.0
STOP_SPEED = 20.0

def has_arrived(distance_error, speed):
    return distance_error < STOP_DISTANCE and abs(speed) < STOP_SPEED

def d_gt(log, tt, min_dist=40.0):
    i = min(int(round(tt / (log["t"][1] - log["t"][0]))), len(log["dist"]) - 1)
    return log["dist"][i] > min_dist

def run_simulation(mode, start=(100.0, 300.0, 0.0), target=(400.0, 400.0),
                   t_max=20.0, dt=1.0 / 60.0, params=None):
    robot = DifferentialDriveRobot(*start, params=params)
    ctrl = ControllerSet(mode)
    dx, dy = target[0] - robot.x, target[1] - robot.y
    ctrl.reset(math.hypot(dx, dy),
               wrap_angle(math.atan2(dy, dx) - robot.theta))

    log = {k: [] for k in ("t", "x", "y", "dist", "head", "v", "omega")}
    t, arrived_at = 0.0, None
    while t < t_max:
        v_cmd, w_cmd, e_d, e_h = ctrl.compute(robot, target, dt)
        if has_arrived(e_d, robot.speed):
            robot.stop()
            arrived_at = t
            for k, val in zip(log, (t, robot.x, robot.y, e_d, e_h, 0.0, 0.0)):
                log[k].append(val)
            break
        v, w = robot.step(v_cmd, w_cmd, dt)
        for k, val in zip(log, (t, robot.x, robot.y, e_d, e_h, v, w)):
            log[k].append(val)
        t += dt

    sx, sy = start[0], start[1]
    ux, uy = target[0] - sx, target[1] - sy
    norm = math.hypot(ux, uy)
    ux, uy = ux / norm, uy / norm
    past = [((x - target[0]) * ux + (y - target[1]) * uy) for x, y in zip(log["x"], log["y"])]
    overshoot = max(0.0, max(past))
    path_len = sum(math.hypot(log["x"][i] - log["x"][i - 1], log["y"][i] - log["y"][i - 1])
                   for i in range(1, len(log["x"])))
    first_in = next((tt for tt, d in zip(log["t"], log["dist"]) if d < 10.0), None)
    cross = [abs((x - sx) * uy - (y - sy) * ux) for x, y in zip(log["x"], log["y"])]
    steady = [abs(h) for tt, h in zip(log["t"], log["head"]) if tt > 1.5 and d_gt(log, tt)]
    metrics = {
        "arrival_time": arrived_at,
        "reach_10px_time": first_in,
        "final_error": log["dist"][-1],
        "overshoot": overshoot,
        "peak_speed": max(log["v"]),
        "path_length": path_len,
        "max_abs_heading_deg": math.degrees(max(abs(h) for h in log["head"])),
        "max_cross_track": max(cross),
        "mean_heading_deg": math.degrees(sum(steady) / len(steady)) if steady else 0.0,
    }
    return log, metrics
