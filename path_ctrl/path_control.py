import numpy as np
import math
import time
k = 0.1  # 前视距离系数
Lfc = 2.0  # 前视距离
Kp = 1.0  # 速度P控制器系数
dt = 0.1  # 时间间隔，单位：s
L = 5.8  # 车辆轴距，单位：m

    #  设置目标路点
#生成numpy数组 大小为4*2
# print("target_p:",target_p)


def delete_first_line(filename):
    try:
        with open(filename, 'r') as f:
            lines = f.readlines()
        with open(filename, 'w') as f:
            f.writelines(lines[1:])
    except FileNotFoundError:
        print(f"File '{filename}' not found.")

def get_txt_lines(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
    line_number = len(lines)
    return line_number

def get_car_target(state_x,state_y,file_path):
    target_positions = np.loadtxt(file_path)
    target_positions = np.array(target_positions)
    line_number = get_txt_lines(file_path)
    if line_number == 0:
        return None
    elif line_number == 1:
        x = target_positions[1] - state_y
        y = target_positions[0] - state_x
        distance = math.sqrt(x**2 + y**2)
        print("distance:",distance)
        if distance < 0.5:
            #移除第一个元素
            delete_first_line(file_path)
            # time.sleep(2)
        target = target_positions
        print("target",target)
        return target
    else:
        x = target_positions[0][1] - state_y
        y = target_positions[0][0] - state_x
        distance = math.sqrt(x**2 + y**2)
        print("distance:",distance)
        if distance < 0.5:
            #移除第一个元素
            delete_first_line(file_path)
            # time.sleep(2)
        # print("target_positions:",target_positions)
        target = target_positions[0]
        print("target",target)
        return target
    
car1_target_speed = 5  # [m/s]

class VehicleState:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

class DroneState:
    def __init__(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

def PControl(target_speed,state):
    if target_speed > state.v:
        a = 1
    elif target_speed < state.v:
        a = -1
    if target_speed == 0:
        a = 0
    return a

def angle_control(state, target_position):
    tx = target_position[0]
    ty = target_position[1]
    rotation_target = math.atan2(tx - state.x, ty - state.y)
    rotation_target = rotation_target * 180 / math.pi
    self_angle = state.yaw
    if self_angle > 180:
        self_angle -= 360
    alpha = rotation_target - self_angle
    if alpha > 180:
        alpha -= 360
    if alpha < -180:
        alpha += 360
    print("rotation_target:",rotation_target,"self",self_angle,"alpha",alpha)
    if alpha > 5:
        alpha = -1
    elif alpha < -5:
        alpha = 1
    else:
        alpha = 0
    return alpha

def get_drone_target(state_x,state_y,state_z,file_path):
    target_positions = np.loadtxt(file_path)
    target_positions = np.array(target_positions)
    line_number = get_txt_lines(file_path)
    if line_number == 0:
        target = None
    elif line_number == 1:
        x = target_positions[0] - state_x
        y = target_positions[1] - state_y
        z = target_positions[2] - state_z
        distance = math.sqrt(x ** 2 + y ** 2 + z ** 2)
        print("drone_distance:",distance)
        if distance < 0.5:
            #移除第一个元素
            delete_first_line(file_path)
            # time.sleep(2)
        target = target_positions
        print("drone_target",target)
    else:
        x = target_positions[0][0] - state_x
        y = target_positions[0][1] - state_y
        z = target_positions[0][2] - state_z
        distance = math.sqrt(x ** 2 + y ** 2 + z ** 2)
        if distance < 6:
            #移除第一个元素
            delete_first_line(file_path)
        target = target_positions[0]
        print("drone_target",target)
    return target

def drone_move_control(state, target_position):
    control = np.zeros(4)
    tx = target_position[0]
    ty = target_position[1]
    tz = target_position[2]
    rotation_target = math.atan2(tx - state.x, tz - state.z)
    rotation_target = rotation_target * 180 / math.pi
    self_angle = state.yaw
    if self_angle > 180:
        self_angle -= 360
    alpha = rotation_target - self_angle
    if alpha > 180:
        alpha -= 360
    if alpha < -180:
        alpha += 360
    if abs(alpha) > 1:
        if alpha > 1:
            control[3] = -1
        elif alpha < -1:
            control[3] = 1
        else :
            control[3] = 0
    elif abs(state.y - ty) > 3:
        if state.y > ty:
            control[2] = -1
        elif state.y < ty:
            control[2] = 1
        else :
            control[2] = 0
    if abs(alpha) <30:
        distance = math.sqrt((state.x - tx) ** 2 + (state.z - tz) ** 2)
        if distance > 0.5:
                control[0] = 1
        else :
                control[0] = 0
    return control

def car1_control(rev):

    # 设置车辆的出事状态
    car1_state = VehicleState(x=rev[0], y=rev[2], yaw=rev[4], v=rev[6])
    data = np.zeros(3)
    target_position = get_car_target(car1_state.x,car1_state.y,'D:/LLM_unity/001.txt')
    if target_position is None:
        car1_target_speed = 0
        data[0] = 0
        data[1] = 0
        data[2] = 1
    else :
        car1_target_speed = 5    
        ai = PControl(car1_target_speed, car1_state)
        di = angle_control(car1_state, target_position)
        data[0] = ai
        data[1] = di
        data[2] = 0
    return data

def car2_control(rev):
    car2_state = VehicleState(x=rev[9], y=rev[11], yaw=rev[13], v=rev[15])
    data = np.zeros(3)
    target_position = get_car_target(car2_state.x,car2_state.y,'D:/LLM_unity/002.txt')
    if target_position is None:
        car2_target_speed = 0
        data[0] = 0
        data[1] = 0
        data[2] = 1
    else :
        car2_target_speed = 5    
        ai = PControl(car2_target_speed, car2_state)
        di = angle_control(car2_state, target_position)
        data[0] = ai
        data[1] = di
        data[2] = 0
    return data

def drone1_control(rev):
    drone1_state = DroneState(x = rev[18], y = rev[19], z = rev[20], yaw = rev [22])
    data = np.zeros(4)
    target_position = get_drone_target(drone1_state.x, drone1_state.y, drone1_state.z, 'D:/LLM_unity/003.txt')
    if target_position is None:
        return data
    else :
        xy = drone_move_control(drone1_state, target_position)
        data = xy
        return data

def drone2_control(rev):
    drone2_state = DroneState(x = rev[26], y = rev[27], z = rev[28], yaw = rev [30])
    data = np.zeros(4)
    target_position = get_drone_target(drone2_state.x, drone2_state.y, drone2_state.z, 'D:/LLM_unity/004.txt')
    if target_position is None:
        return data
    else :
        xy = drone_move_control(drone2_state, target_position)
        data = xy
        return data
    
def main_control(rev):
    data_send = np.zeros(14)
    data_send[0:3] = car1_control(rev)
    data_send[3:6] = car2_control(rev)
    data_send[6:10] = drone1_control(rev)
    data_send[10:] = drone2_control(rev)
    return data_send
    
# if __name__ == '__main__':
#     main_control()

