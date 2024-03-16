from . import data_struct as struct

import time
import numpy as np

"""
用 Vehicle 来统一管理无人车 SwarmAENode 对象
"""
class Vehicle:
    def __init__(self, Client, vehicle_name, vehicle_no, vehicle_type="四轮车"):
        # 设置 Client
        self.Client_ = Client
        
        # 定义车辆基本信息
        self.type_ = vehicle_type  # 节点类型
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
    
    def get_imu_data(self):
        vx, vy, vz, v, ax, ay, az, a, g, p, q, r, _,  = self.Vehicle_.get_velocity()
        vehicle_imu = struct.ImuData(vx, vy, vz, v, ax, ay, az, a, g, p, q, r)
        return vehicle_imu  
    
    def get_transform(self) -> struct.Transform:
        yaw, _, _, _, _ = self.Vehicle_.get_attitude()
        x, y, _, _ = self.Vehicle_.get_location()
        vehicle_pose = struct.Transform(x, y, yaw)
        return vehicle_pose
    
    def apply_control(self, steer, throttle, brake, hand_brake):
        # 油门为正是前进，油门为负是倒车
        throttle = min(throttle,  1)
        throttle = max(throttle, -1)

        # 无论前进还是倒车，刹车都为正
        brake = min(brake, 1)
        brake = max(brake, 0)

        # 转向的取值范围是 [-1, 1]
        steer = min(steer,  1)
        steer = max(steer, -1)

        # 调用接口控制车辆
        # 挡位设为 0 即可，不用调整
        _, status_code = self.Vehicle_.apply_vehicle_control(throttle, steer, brake, hand_brake, 0)

        return status_code

"""
用 Drone 来统一管理无人机 SwarmAENode 对象
"""    
class Drone:
    def __init__(self, Client, drone_name, drone_no, drone_type="四旋翼") -> float:
        # 设置 Client
        self.Client_ = Client
        
        # 定义无人机基本信息
        self.type_ = drone_type  # 节点类型
        self.name_ = drone_name  # 节点名称
        self.no_   = drone_no    # 节点在队伍中的编号
        
        # 创建无人机节点
        timestamp = int(round(time.time() * 1000)) 
        _, self.Drone_, _ = self.Client_.register_node(node_type = self.type_, 
                                                       node_name = self.name_,
                                                       node_no   = self.no_  ,
                                                       frame_timestamp = timestamp)    
        
    def apply_control(self, target_x, target_y, target_z, target_v):
        timestamp = int(round(time.time() * 1000))
        status_code, _ = self.Drone_.control_kinetic_simply_global(target_x, target_y, target_z, target_v, timestamp)

        return status_code
    
    def get_transform(self) -> struct.Transform:
        yaw, _, _, _, _ = self.Drone_.get_attitude()
        x, y, z, _ = self.Drone_.get_location()
        drone_pose = struct.Transform(x, y, yaw, z)
        return drone_pose
    
    def get_imu_data(self):
        vx, vy, vz, v, ax, ay, az, a, g, p, q, r, _,  = self.Drone_.get_velocity()
        drone_imu = struct.ImuData(vx, vy, vz, v, ax, ay, az, a, g, p, q, r)
        return drone_imu 