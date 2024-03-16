from lib.common import TransformStruct
from lib.common import ImuStruct

import time
import numpy as np


class Vehicle:
    """
    用 Vehicle 来统一管理 SwarmAENode 对象
    """

    def __init__(self, Client, vehicle_name, vehicle_no):
        # 设置 Client
        self.Client_ = Client

        # 定义车辆基本信息
        self.type_ = "四轮车"  # 节点类型
        self.name_ = vehicle_name  # 节点名称
        self.no_ = vehicle_no  # 节点在队伍中的编号

        # 创建车辆节点
        timestamp = int(round(time.time() * 1000))
        _, self.Vehicle_, _ = self.Client_.register_node(node_type=self.type_,
                                                         node_name=self.name_,
                                                         node_no=self.no_,
                                                         frame_timestamp=timestamp)

        # 读取车辆尺寸
        vehicle_size = self.Vehicle_.get_size()  # 获取车辆尺寸参数
        self.width_ = vehicle_size['y']  # 车辆宽度
        self.length_ = vehicle_size['x']  # 车辆长度
        self.height_ = vehicle_size['z']  # 车辆高度

        # 读取车辆颜色
        vehicle_rgb = self.Vehicle_.get_color()  # 读取车辆颜色参数
        self.color_ = np.mat([vehicle_rgb['r'],  # 车辆颜色的 [r, g, b] 值
                              vehicle_rgb['g'],
                              vehicle_rgb['b']])

        # 读取车辆的其他属性
        _, _, team, _, model, _ = self.Vehicle_.get_node_info()
        self.team_ = team  # 车辆编队归属（分为红/蓝两队）
        self.model_ = model  # 车辆型号
        self.id_ = self.Vehicle_.get_node_id()  # 车辆唯一标识 id

        ImuData = self.get_imu_data()

    def get_imu_data(self):
        vx, vy, vz, v, ax, ay, az, a, g, p, q, r, _, = self.Vehicle_.get_velocity()
        ImuData = ImuStruct(vx, vy, vz, v, ax, ay, az, a, g, p, q, r)
        return ImuData

    def get_transform(self):
        yaw, pitch, roll, heading, _ = self.Vehicle_.get_attitude()
        x, y, z, _ = self.Vehicle_.get_location()
        TransformData = TransformStruct(roll, pitch, yaw, heading, x, y, z)
        return TransformData

    def longitudinal_control(self, accel, hand_brake):
        accel = min(accel, 1.0)
        accel = max(accel, -1.0)

        if accel != 0:
            hand_brake = 0
        else:
            hand_brake = min(hand_brake, 1.0)
            hand_brake = max(hand_brake, -1.0)

        _, status_code = self.Vehicle_.control_vehicle(accel)
        _, status_code = self.Vehicle_.set_vehicle_brake(hand_brake)

        return status_code

    def lateral_control(self, steer):
        steer = min(steer, 1.0)
        steer = max(steer, -1.0)

        _, status_code = self.Vehicle_.set_vehicle_steer(steer)

        return status_code

    # def control(self, accel, hand_brake, steer):
    #     # 限制控制量
    #     # accel > 0 为前进，accel < 0 为减速 / 倒车
    #     accel = min(accel,  1.0)
    #     accel = max(accel, -1.0)

    #     if accel != 0:
    #         hand_brake = 0
    #     else:
    #         hand_brake = min(hand_brake,  1.0)
    #         hand_brake = max(hand_brake, -1.0)

    #     # 向左打轮为负，向右打轮为正
    #     steer = min(steer,  1.0)
    #     steer = max(steer, -1.0)

    #     # 发送控制指令
    #     _, status_steer      = self.Vehicle_.set_vehicle_steer(steer)
    #     _, status_acc        = self.Vehicle_.control_vehicle(accel)
    #     _, status_hand_brake = self.Vehicle_.set_vehicle_brake(hand_brake)

    #     # 返回控制结果
    #     if ((status_acc        == 200) and
    #         (status_steer      == 200) and
    #         (status_hand_brake == 200)):
    #         status_code = 200
    #     else:
    #         status_code = 500
    #         print("\033[0;31;40mConnection Lost!\033[0m")

    #     return status_code