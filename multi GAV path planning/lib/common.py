class TransformStruct:
    """
    存储位姿信息的结构体
    """
    def __init__(self, roll, pitch, yaw, heading, x, y, z):
        self.roll    = roll
        self.pitch   = pitch
        self.yaw     = yaw
        self.heading = heading
        self.x       = x
        self.y       = y

class ImuStruct:
    """
    存储速度信息的结构体
    """
    def __init__(self, vx, vy, vz, v, ax, ay, az, a, g, omega_pitch, omega_roll, omega_yaw):
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.v  = v
        self.ax = ax
        self.ay = ay
        self.az = az
        self.a  = a
        self.g  = g
        self.omega_pitch = omega_pitch
        self.omega_roll  = omega_roll
        self.omega_yaw   = omega_yaw