import numpy as np
import __init__
import math
import drone_formation.formation as fm

d_v = 100
car_states = []
max_speed = 50
max_add_speed = 25
alpha = 0.001

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
        if distance < 5:
            #移除第一个元素
            delete_first_line(file_path)
            # time.sleep(2)
        target = target_positions
        return target
    else:
        x = target_positions[0][1] - state_y
        y = target_positions[0][0] - state_x
        distance = math.sqrt(x**2 + y**2)
        if distance < 5:
            #移除第一个元素
            delete_first_line(file_path)
        target = target_positions[0]
        return target
    
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
    if alpha > 5:
        alpha = -1
    elif alpha < -5:
        alpha = 1
    else:
        alpha = 0
    return alpha

def car_control(state):
    update(state)
    # 设置车辆的出事状态
    speed_x = state.velocity[0]
    speed_y = state.velocity[2]
    speed = math.sqrt(speed_x**2 + speed_y**2)
    car_state = VehicleState(x=state.position[0], y=state.position[2], yaw=state.rotation[1], v=speed)
    target_position = get_car_target(car_state.x, car_state.y, 'txt_data/00{}.txt'.format(state.id))
    if target_position is None:
        car_target_speed = 0
        state.car_manual_control = [False, False, False, False, True]
    else :
        car_tar_dis = math.sqrt((target_position[0] - car_state.x)**2 + (target_position[1] - car_state.y)**2)
        if car_tar_dis < 10:
            car_target_speed = 5
        elif car_tar_dis < 30:
            car_target_speed = 10
        else:
            car_target_speed = 20   
        di = angle_control(car_state, target_position)
        if di is not 0:
            car_target_speed = 5
        ai = PControl(car_target_speed, car_state)
        if ai == 0:
            front = False 
            back = False
        elif ai == 1:
            front = True
            back = False
        elif ai == -1:
            front = False
            back = True
        if di == 0:
            left = False
            right = False
        elif di == 1:
            left = True
            right = False
        elif di == -1:
            left = False
            right = True
        state.car_manual_control = [front, back, left, right, False]
    return state

n = 12
ob_n = 4
distance = np.zeros((n+1, n+1))
velocity = np.zeros((n+1, 2))
position = np.zeros((n+1, 2))
ob_position = np.zeros((ob_n, 2))
target_distance = np.zeros((n+1, n+1))
clockwise = np.zeros(n+1)
virtual_drone_position = np.zeros((n+1,2))
target_virtual_distance = np.zeros((n+1, 1))
alpha_force = 1000000
F_d = np.array([0,0])


def set_obstacle():
    ob_position[0,0] = -200
    ob_position[0,1] = -425
    ob_position[1,0] = -425
    ob_position[1,1] = -200
    ob_position[2,0] = 200
    ob_position[2,1] = -425
    ob_position[3,0] = 425
    ob_position[3,1] = -200


# force of obstacle
def get_avoid_force(x,y):
    F_d[0] = 0
    F_d[1] = 0
    for j in range(0,ob_n):
        q = 100
        d = math.sqrt((x - ob_position[j,0])**2 + (y - ob_position[j,1])**2)
        if d < q:
            F_d[0] = F_d[0] + alpha_force * (1/d - 1/100) ** 2 * (1/d) * (x - ob_position[j,0])
            F_d[1] = F_d[1] + alpha_force * (1/d - 1/100) ** 2 * (1/d) * (y - ob_position[j,1])

def auto_avoid_obstacle():
    set_obstacle()
    for i in range (2,n+1):
        get_avoid_force(position[i,0],position[i,1])
        velocity[i,0] = velocity[i,0] + F_d[0]
        velocity[i,1] = velocity[i,1] + F_d[1]

def target_distance_init():
    for i in range(0,n):
        for j in range(0,n):
            target_distance[i,j] = -1
    for i in range(1,n):
        target_distance[i,i] = 0

    target_virtual_distance[2] = 0
    target_virtual_distance[6] = 0
    target_distance[3,0] = 141.4
    target_distance[3,2] = 200
    target_distance[4,0] = 141.4
    target_distance[4,2] = 200
    target_distance[5,0] = 141.4
    target_distance[5,2] = 282.4
    target_distance[7,1] = 141.4
    target_distance[7,6] = 200
    target_distance[8,1] = 141.4
    target_distance[8,6] = 200
    target_distance[9,1] = 141.4
    target_distance[9,6] = 282.4


def clockwise_init():
    clockwise[1] = 0
    clockwise[2] = 1
    clockwise[3] = -1
    clockwise[4] = 1
    clockwise[5] = -1
    clockwise[6] = 1
    clockwise[7] = -1
    clockwise[8] = 1
    clockwise[9] = -1

def update(state):
    velocity[int(state.id),0] = state.velocity[0]
    velocity[int(state.id),1] = state.velocity[2]
    position[int(state.id),0] = state.position[0]
    position[int(state.id),1] = state.position[2]

    if int(state.id) == 0:
        virtual_drone_position[0,0] = state.position[0] + d_v
        virtual_drone_position[0,1] = state.position[2] + d_v
    if int(state.id) == 1:
        virtual_drone_position[1,0] = state.position[0] - d_v
        virtual_drone_position[1,1] = state.position[2] + d_v
    for k in range(0, n+1):
        for j in range(0, n+1):
            distance[k,j] = math.sqrt((position[k,0] - position[j,0])**2 + (position[k,1] - position[j,1])**2)

def get_first_follower_v():
    y1, y2, d1, d2 = fm.transforming(target_virtual_distance[2], target_distance[2,2], position[2,:], virtual_drone_position[0,:])
    s = fm.cal_s(y1, y2, d1, d2)
    Rs = np.array([[s, clockwise[2] * math.sqrt(1 - s*s)],[-clockwise[2] * math.sqrt(1 - s*s), s]])
    velocity[2,:] = -distance[2,0] ** 2 * (position[2,:] - y1).T - np.dot(d1 * distance[2,0] * Rs, np.array((y1 - y2).T))    

def get_second_follower_v():
    y1, y2, d1, d2 = fm.transforming(target_virtual_distance[6], target_distance[6,6], position[6,:], virtual_drone_position[1,:])
    s = fm.cal_s(y1, y2, d1, d2)
    Rs = np.array([[s, clockwise[6] * math.sqrt(1 - s*s)],[-clockwise[6] * math.sqrt(1 - s*s), s]])
    velocity[6,:] = -distance[6,1] ** 2 * (position[6,:] - y1).T - np.dot(d1 * distance[6,1] * Rs, np.array((y1 - y2).T))    

def control():
    get_first_follower_v()
    for k in range(3,6):
        y1, y2, d1, d2 = fm.transforming(target_distance[k,0], target_distance[k,2], position[0,:], position[2,:])
        s = fm.cal_s(y1, y2, d1, d2)
        Rs = np.array([[s, clockwise[k] * math.sqrt(1 - s*s)],[-clockwise[k] * math.sqrt(1 - s*s), s]])
        velocity[k,:] = -distance[2,0] ** 2 * (position[k,:] - y1).T - np.dot(d1 * distance[2,0] * Rs, np.array((y1 - y2).T))
    get_second_follower_v()
    for k in range(7,10):
        y1, y2, d1, d2 = fm.transforming(target_distance[k,1], target_distance[k,6], position[1,:], position[6,:])
        s = fm.cal_s(y1, y2, d1, d2)
        Rs = np.array([[s, clockwise[k] * math.sqrt(1 - s*s)],[-clockwise[k] * math.sqrt(1 - s*s), s]])
        velocity[k,:] = -distance[6,1] ** 2 * (position[k,:] - y1).T - np.dot(d1 * distance[6,1] * Rs, np.array((y1 - y2).T))

def small():
    for i in range(1,n+1):
        velocity[i,0] = alpha * velocity[i,0] 
        velocity[i,1] = alpha * velocity[i,1]

def check():

    for i in range(0,n+1):
        if velocity[i,0] > max_speed:
            velocity[i,0] = max_speed
        if velocity[i,0] < -max_speed:
            velocity[i,0] = -max_speed
        if velocity[i,1] > max_speed:
            velocity[i,1] = max_speed
        if velocity[i,1] < -max_speed:
            velocity[i,1] = -max_speed
        

def drone_control():
    control()
    small()
    check()
    auto_avoid_obstacle()
    check()

def main_control(agents):
    # get the length of agents
    n = len(agents)
    for i in range(0,n):
        update(agents[i])
    drone_control()
    for i in range(0,n):
        if agents[i].type == 'car':
            agents[i] = car_control(agents[i])
        else:
            if abs(velocity[int(agents[i].id),0] - agents[i].drone_speed_control[0]) > max_add_speed:
                if velocity[int(agents[i].id),0] > agents[i].drone_speed_control[0]:
                    agents[i].drone_speed_control[0] = agents[i].drone_speed_control[0] + max_add_speed
                else:
                    agents[i].drone_speed_control[0] = agents[i].drone_speed_control[0] - max_add_speed
            else:
                agents[i].drone_speed_control[0] = velocity[int(agents[i].id),0]
            if abs(velocity[int(agents[i].id),1] - agents[i].drone_speed_control[2]) > max_add_speed:
                if velocity[int(agents[i].id),1] > agents[i].drone_speed_control[2]:
                    agents[i].drone_speed_control[2] = agents[i].drone_speed_control[1] + max_add_speed
                else:
                    agents[i].drone_speed_control[2] = agents[i].drone_speed_control[0] - max_add_speed
            else:
                agents[i].drone_speed_control[2] = velocity[int(agents[i].id),1]
    return agents
    

