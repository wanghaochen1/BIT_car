from swarmae.SwarmAEClient import SwarmAEClient
import time
import numpy as np

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

class Vehicle:
    """
    用 Vehicle 来统一管理 SwarmAENode 对象
    """
    def __init__(self, Client, vehicle_name, vehicle_no):
        # 设置 Client
        self.Client_ = Client
        
        # 定义车辆基本信息
        self.type_ = "四轮车"       # 节点类型
        self.name_ = vehicle_name  # 节点名称
        self.no_   = vehicle_no    # 节点在队伍中的编号
        
        # 创建车辆节点
        timestamp = int(round(time.time() * 1000))
        _, self.Vehicle_, _ = self.Client_.register_node(node_type = self.type_, 
                                                         node_name = self.name_,
                                                         node_no   = self.no_  ,
                                                         frame_timestamp = timestamp)    
        
        # 读取车辆尺寸
        vehicle_size  = self.Vehicle_.get_size() # 获取车辆尺寸参数
        self.width_   = vehicle_size['y']        # 车辆宽度
        self.length_  = vehicle_size['x']        # 车辆长度
        self.height_  = vehicle_size['z']        # 车辆高度
        
        # 读取车辆颜色
        vehicle_rgb = self.Vehicle_.get_color()  # 读取车辆颜色参数
        self.color_ = np.mat([vehicle_rgb['r'],  # 车辆颜色的 [r, g, b] 值 
                              vehicle_rgb['g'], 
                              vehicle_rgb['b']])
        
        # 读取车辆的其他属性
        _, _, team, _, model, _ = self.Vehicle_.get_node_info()
        self.team_  = team  # 车辆编队归属（分为红/蓝两队） 
        self.model_ = model # 车辆型号
        self.id_    = self.Vehicle_.get_node_id()  # 车辆唯一标识 id

        ImuData = self.get_imu_data()
        
    
    def get_imu_data(self):
        vx, vy, vz, v, ax, ay, az, a, g, p, q, r, _,  = self.Vehicle_.get_velocity()
        ImuData = ImuStruct(vx, vy, vz, v, ax, ay, az, a, g, p, q, r)
        return ImuData  
    
    def get_transform(self):
        yaw, pitch, roll, heading, _ = self.Vehicle_.get_attitude()
        x, y, z, _ = self.Vehicle_.get_location()
        TransformData = TransformStruct(roll, pitch, yaw, heading, x, y, z)
        return TransformData
    
    def longitudinal_control(self, accel, hand_brake):
        accel = min(accel,  1.0)
        accel = max(accel, -1.0)
        
        if accel != 0:
            hand_brake = 0
        else:
            hand_brake = min(hand_brake,  1.0)
            hand_brake = max(hand_brake, -1.0)
        
        _, status_code = self.Vehicle_.control_vehicle(accel)
        _, status_code = self.Vehicle_.set_vehicle_brake(hand_brake)    
        
        return status_code
        
    def lateral_control(self, steer):
        steer = min(steer,  1.0)
        steer = max(steer, -1.0)            
        
        _, status_code = self.Vehicle_.set_vehicle_steer(steer)
        
        return status_code

class autoController:
    def __init__(self, Vehicle):
        self.prev_accel_ = 0
        self.prev_brake_ = 0
        self.prev_steer_ = 0

        self.accel_ = 0
        self.brake_ = 0
        self.steer_ = 0

        self.hand_brake_ = 0

        self.Vehicle_ = Vehicle

        print("Auto Control Ready.")

    def read_keyboard(self, forward, left, over):#向前加速度Forward 向左Left，向后加速度forward为负数 向右left加速度为负
        # W 和 S 分别表示加速和减速

        if forward > 0:                 #加速
            if self.prev_accel_ < 0:
                self.prev_accel_ = 0

            accel = self.prev_accel_ + 0.1
            accel = min(accel, 1.0)     #加速度最小为1，这里应该可以去修改
            self.accel_ = accel
            self.prev_accel_ = self.accel_
        elif forward < 0:               #减速
            if self.prev_accel_ > 0:
                self.prev_accel_ = 0

            accel = self.prev_accel_ - 0.1
            accel = max(accel, -1.0)
            self.accel_ = accel
            self.prev_accel_ = self.accel_
        else:
            self.prev_accel_ = 0        #匀速
            self.accel_ = 0

        steer_increment = 0.1           #转向角度

        # A 和 D 分别表示向左和向右转向
        if left > 0 :                   #向左转向
            if self.prev_steer_ > 0:
                self.prev_steer_ = 0
            else:
                self.prev_steer_ -= steer_increment
        elif left < 0:                  #向右转向
            if self.prev_steer_ < 0:
                self.prev_steer_ = 0
            else:
                self.prev_steer_ += steer_increment
        else:
            self.prev_steer_ = 0.0      #保持直行

        self.steer_ = min(1.0, max(-1.0, self.prev_steer_))

        break_flag = over      #over 为bool,继续控制就是False

        # 输出控制量
        self.Vehicle_.longitudinal_control(self.accel_, self.hand_brake_)
        self.Vehicle_.lateral_control(self.steer_)

        print("\rControl Input: steer %.2f, accel %.2f, hand_brake %d" % (self.steer_, self.accel_, self.hand_brake_),
              end="")

        return break_flag
    