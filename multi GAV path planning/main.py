import time
import cv2
from multiprocessing import Process
import threading
# 仿真软件官方的工具包
from swarmae.SwarmAEClient import SwarmAEClient

# 自己封装的函数和接口
import lib.swarm_api as api
import lib.Astar as Astar
import lib.frenet_optimal_planner as fop
import lib.utils as utils
import lib.data_struct as struct
import lib.controller as controller
import lib.param_parser as param_parser
from lib.utils import *

end_x = 1800
end_y = 2010
STOP = 0
VEHICLE_START_TIME = 5


class cVehicle:
    def __init__(self, vehicle, maps, world_offset, end_x, end_y, start_time=5, number: int = 1):
        self.vehicle = vehicle
        start_x = int(vehicle.get_transform().x_ - world_offset[0])
        start_y = int(vehicle.get_transform().y_ - world_offset[1])
        print("car{}: start at ({}, {}), end at ({}, {})".format(number, start_x, start_y, end_x, end_y))
        # self.path_planner = Astar.ClassAStar(start_x,
        #                                 start_y,
        #                                 end_x,
        #                                 end_y,
        #                                 maps,
        #                                 step_size=5,
        #                                 search_range=20,
        #                                 scale=1)
        # self.path = self.path_planner.update_planning()
        self.path = Astar.Astar(maps, start_x, start_y, end_x, end_y)
        informap = cv2.cvtColor(maps, cv2.COLOR_GRAY2BGR)

        for i in range(len(self.path)):  # 整数型画路径
            cv2.circle(informap, self.path[i], 2, (0, 0, 200), -1)
        cv2.circle(informap, (start_x, start_y), 15, (0, 255, 0), -1)  # 起点
        cv2.circle(informap, (end_x, end_y), 15, (0, 0, 255), -1)  # 终点
        cv2.imwrite("informap{}.png".format(number), informap)

        self.path = path_to_ue4(self.path, world_offset)
        # print(self.path)
        # # 提取x和y坐标点
        # x = [point[0] for point in self.path]
        # y = [point[1] for point in self.path]

        # # 创建曲线图像
        # plt.plot(x, y, marker='o', linestyle='-')

        # # 添加标签
        # plt.title('path')
        # plt.xlabel('X')
        # plt.ylabel('Y')

        # # 显示图像
        # plt.show()
        self.end_x = end_x
        self.end_y = end_y
        self.vehicle_pose = self.vehicle.get_transform()
        self.vehicle.apply_control(0, 1, 0, False)
        time.sleep(start_time)
        # vehicle1.apply_control(0, 0, 1, False)
        self.vehicle_pose = self.vehicle.get_transform()

        # 创建用于补全全局路径的 ReferencePath
        reference_x = np.mat([x[0] for x in self.path]).T
        reference_y = np.mat([x[1] for x in self.path]).T
        self.ref_path = fop.ReferencePath(reference_x, reference_y, 0.1)
        

        # 创建用于碰撞检测的 VehicleGeometry
        self.car_length = self.vehicle.length_
        self.car_width = self.vehicle.width_
        self.vehicle_geometry = fop.VehicleGeometry(self.car_length, self.car_width)

        self.vehicle_pose = self.vehicle.get_transform()
        self.vehicle_pose.theta_ = self.vehicle_pose.theta_ * math.pi / 180
        self.vehicle_imu = self.vehicle.get_imu_data()
        initial_v = self.vehicle_imu.v
        initial_a = self.vehicle_imu.a
        initial_kappa = 0

        parameters = param_parser.Parameters('./config/params.ini')
        self.vehicle_state = struct.State(self.vehicle_pose, initial_v, initial_a, initial_kappa)
        self.stanley = controller.Stanley(parameters.K_)
        self.pid = controller.PID(parameters.Kp_, parameters.Ki_, parameters.Kd_)

        #    2.4 读取车辆当前位置，通过 transform_cartesian_to_frenet 初始化车辆当前的 frenet 坐标
        self.frenet_state = fop.transform_cartesian_to_frenet(self.vehicle_state, self.ref_path)
        #    2.5 创建局部规划器 FrenetOptimalPlanner，并从当前坐标规划一帧局部轨迹
        self.planner = fop.FrenetOptimalPlanner(self.ref_path, parameters)
        self.optimal_trajectory, self.valid_trajectory, self.trajectory_list = self.planner.update_planning(
            self.frenet_state, self.vehicle_geometry, [])

        self.wheel_base = self.vehicle.length_ * 0.85  # 0.85是学长推荐的估计值

        self.dist = utils.distance(self.vehicle_pose.x_, self.vehicle_pose.y_, self.end_x, self.end_y)

        # 创建记录数据的空数组
        self.actual_x_list = []
        self.actual_y_list = []

        # 初始化采样时间
        self.prev_time = time.time()
        self.dt = 0.1

    def vehicle_update(self):
        if self.valid_trajectory != [] and len(self.optimal_trajectory.s_) > 2:
            pass
        else:
            # 到这儿说明局部规划寄了
            print("\nNo valid path")
            return STOP

def vehicle_update(car: cVehicle):
    while True:
        if car.valid_trajectory != [] and len(car.optimal_trajectory.s_) > 2:
            pass
        else:
            # 到这儿说明局部规划寄了
            print("\nNo valid path")
            return STOP

        # 跑到终点了
        if car.optimal_trajectory.s_[0] >= car.ref_path.interp_s_[-1]:
            # 先踩几秒钟刹车
            car.vehicle.apply_control(steer=0, throttle=0, brake=0.8, hand_brake=False)
            time.sleep(3)
            # 然后拉手刹制动
            car.vehicle.apply_control(steer=0, throttle=0, brake=0, hand_brake=True)
            print("\nFinish!")
            return STOP

        # 记录数据
        car.actual_x_list.append(car.vehicle_pose.x_)
        car.actual_y_list.append(car.vehicle_pose.y_)

        # 实时绘图
        # plot_helper(vehicle_pose1, ref_path, vehicle_geometry, actual_x_list, actual_y_list)

        # 寻找车辆前轴到局部路径的参考点
        front_x = car.vehicle_state.pose_.x_ + car.wheel_base * math.cos(car.vehicle_state.pose_.theta_)
        front_y = car.vehicle_state.pose_.y_ + car.wheel_base * math.sin(car.vehicle_state.pose_.theta_)

        # 找到距离车辆前轴最近的点
        dx = np.ravel(car.optimal_trajectory.x_) - front_x
        dy = np.ravel(car.optimal_trajectory.y_) - front_y
        dist = np.abs(dx ** 2 + dy ** 2)
        min_dist_idx = np.argmin(dist)

        # 通过该点设置目标点
        target_x = car.optimal_trajectory.x_[min_dist_idx, 0]
        target_y = car.optimal_trajectory.y_[min_dist_idx, 0]
        target_theta = car.optimal_trajectory.theta_[min_dist_idx, 0]
        target_v = car.optimal_trajectory.v_[min_dist_idx, 0]
        target_pose = struct.Transform(target_x, target_y, target_theta)

        # 计算控制量
        # 将参考点输入给 Stanley 控制器，计算出转向角 steer
        delta = car.stanley.update_control(target_pose, car.vehicle_state)
        # 归一化？防正弦波发散
        delta = delta / 2
        # delta = delta / (math.pi / 6)
        # 将参考速度输入给 PID 控制器，计算出 accel
        accel = car.pid.update_control(target_v, car.vehicle_state.v_, car.dt)
        # 当 accel 为正时，throttle = accel，brake = 0
        if accel > 0:
            throttle = accel
            brake = 0
        # 当 accel 为负时，throttle = 0, brake = -accel
        else:
            throttle = 0
            brake = -accel
        # 当需要转向（有正弦波趋势）时，减速
        # print("current velocity", car.vehicle_state.v_)
        if bool(abs(delta) > 0.1) & bool(car.vehicle_state.v_ > 10):
            car.vehicle.apply_control(0, -1, 1, False)
            time.sleep(0.1)
        # 调用 Vehicle 的 apply_control 接口，输入油门、转向等信息，控制车辆移动
        # print("current delta:", delta)
        car.vehicle.apply_control(delta, throttle, brake, False)
        # 更新采样时间
        car.dt = time.time() - car.prev_time
        if car.dt < 0.1:
            time.sleep(0.1 - car.dt)
            car.dt = 0.1
        car.prev_time = time.time()
        # 获取车辆当前位置
        car.vehicle_pose = car.vehicle.get_transform()
        car.vehicle_pose.theta_ = car.vehicle_pose.theta_ * math.pi / 180
        # print("current_theta", vehicle_pose1.theta_)
        car.vehicle_imu = car.vehicle.get_imu_data()
        kappa = math.tan(delta) / car.wheel_base
        car.vehicle_state = struct.State(car.vehicle_pose, car.vehicle_imu.v, car.vehicle_imu.a, kappa)

        # 计算车辆当前位置到局部路径的距离
        dx = np.ravel(car.optimal_trajectory.x_) - car.vehicle_pose.x_
        dy = np.ravel(car.optimal_trajectory.y_) - car.vehicle_pose.y_
        dist = np.abs(dx ** 2 + dy ** 2)

        # 从上一帧轨迹中寻找当前位置的投影
        min_dist_idx = np.argmin(dist)
        car.frenet_state = struct.FrenetState(car.optimal_trajectory.s_[min_dist_idx, 0],
                                            car.optimal_trajectory.s_dot_[min_dist_idx, 0],
                                            car.optimal_trajectory.s_ddot_[min_dist_idx, 0],
                                            car.optimal_trajectory.d_[min_dist_idx, 0],
                                            car.optimal_trajectory.d_dot_[min_dist_idx, 0],
                                            car.optimal_trajectory.d_ddot_[min_dist_idx, 0])

        car.optimal_trajectory, car.valid_trajectory, car.trajectory_list = car.planner.update_planning(car.frenet_state,
                                                                                                        car.vehicle_geometry,
                                                                                                        [])

def main(i: int):
    # 连接仿真环境
    AeClient = SwarmAEClient(ue_ip="localhost", ue_port=2000)

    # 读取地图比例及偏差
    _, AeGame, _ = AeClient.get_game()
    AeGame.stage_start('reconnasissance_start')
    AeGame.stage_complete('reconnaissance_end')
    AeGame.stage_start('vau_reconnasissance_start')
    AeGame.stage_complete('vau_reconnaissance_end')
    img, scale, world_offset, _, _ = AeGame.get_road_network()
    maps = cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2GRAY)
    imgGaussBlur = cv2.GaussianBlur(maps, ksize=(3, 3), sigmaX=2)
    binary_Erode = cv2.erode(imgGaussBlur, kernel=np.ones((10, 10), dtype=np.uint8))
    maps = binary_Erode


    # 获取三辆车的信息
    vehicle = api.Vehicle(AeClient,
                           vehicle_name="李四",
                           vehicle_no=i+1)

    car = cVehicle(vehicle, maps, world_offset, end_x, end_y, VEHICLE_START_TIME*(1+i), i+1)
    vehicle_update(car)


if __name__ == '__main__':
    process_list = []
    for i in range(3):
        p = Process(target=main, args=(i,)) #实例化进程对象
        print('创建进程{}'.format(i+1))
        p.start()
        process_list.append(p)
        time.sleep(5)

    for i in process_list:
        p.join()
        print('结束进程'+str(i))

    print('结束测试')
    
