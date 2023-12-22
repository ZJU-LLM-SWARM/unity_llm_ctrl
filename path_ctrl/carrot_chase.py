import numpy as np
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint, solve_ivp
import time

'''
使用示例：
初始化：
waypoints = np.array([[0, 0, 0], [300, 0,300], [150, 0, 600], [-300, 0, 600], [0, 0, 0], [300, 0, 0], [0, 0, 0]])
chaser = carrot_chase.Carrotwaypointchase(waypoints)
调用：
chaser.carrot_waypoint_chase(agents[2],velocity=30)
会直接修改速度控制命令
'''

last_theta = None
last_theta_u = None
last_si = None
last_si_d = None
def ode_carrot_chase(t, y, w1_x, w1_y, w2_x, w2_y,v = 10, delta = 10, k = 60):
    '''
    Carrot Chase based Path Following
    waypoint: [w1_x, w1_y] -> [w2_x, w2_y]
    v: velocity fixed
    delta: lookahead distance
    k: gain
    '''
    # Initializing Params
    global last_theta, last_theta_u,last_si, last_si_d
    dydt = np.zeros(3)
    ugv_x = y[0]
    ugv_y = y[1]
    si = y[2]
    theta = []
    si_d = []
    # XY Controller 
    # Computing the position error
    # Distance of point to line (UGV Position - Desired path)
    R_u = np.sqrt((w1_x - ugv_x)**2 + (w1_y - ugv_y)**2)

    theta = np.arctan2((w2_y - w1_y),(w2_x - w1_x))
    theta_u = np.arctan2(ugv_y - w1_y,ugv_x - w1_x)

    theta = check_atan2(last_theta,theta)
    theta_u = check_atan2(last_theta_u,theta_u)
    last_theta = theta
    last_theta_u = theta_u

    beta = theta - theta_u
    R = np.sqrt(R_u**2 - (R_u*np.sin(beta))**2)#投影距离
    x_target = w1_x + (R+delta)*np.cos(theta)
    y_target = w1_y + (R+delta)*np.sin(theta)

    si_d = np.arctan2(y_target - ugv_y, x_target - ugv_x)
    si_d = check_atan2(last_si_d,si_d)
    last_si_d = si_d
    si = check_atan2(last_si,si)
    last_si = si

    u = k*(si_d - si) #后续应该添加控制量限制

    # Finally heading angle
    si_dot = u

    # STATE EQUATIONS
    dydt[0] = v*np.cos(si)  # y[0] -> uav_x
    dydt[1] = v*np.sin(si)  # y[1] -> uav_y
    dydt[2] = si_dot  # y[2] -> si

    return dydt


def check_atan2(last,now):
    if last is not None:
        diff_theta = now - last
        if diff_theta > np.pi:
            now -= 2 * np.pi
        elif diff_theta < -np.pi:
            now += 2 * np.pi
    return now
#waypoints = np.array([[0, 0, 0], [300, 300, 0], [150, 600, 0], [-300, 600, 0], [0, 0, 0], [300, 0, 0], [0, 0, 0]])

class Carrotwaypointchase:
    def __init__(self,waypoints):
        self.waypoints = waypoints
        self.waypoints_num = len(waypoints)
        self.waypoints_index = 1 #0-1点开始，不会再从0开始，而是在0-1连线上追1点
        self.count = 0
    def carrot_waypoint_chase(self,unity_state,threshold = 5,velocity=10,delta=10,k=60):
        '''
        Carrot Chase based Path Following
        unity_state: 只传一个agent
        threshold: 距离目标点的阈值
        velocity:预期的固定速度
        delta: lookahead distance
        k: gain
        '''
        self.count += 1
        t_eval = np.linspace(0, 0.2, 6)#
        curr_x = unity_state.position[0]
        curr_y = unity_state.position[2]#导入环境pos
        curr_si = np.sqrt(unity_state.velocity[0]**2+unity_state.velocity[2]**2)
        wp_1 = self.waypoints[self.waypoints_index-1, :]
        w1_x, w1_y = wp_1[0], wp_1[2]
        wp_2 = self.waypoints[self.waypoints_index, :]
        w2_x, w2_y = wp_2[0], wp_2[2]#获取当前目标点
        dist_wp = np.sqrt((w2_x - curr_x)**2 + (w2_y - curr_y)**2)#更新error

        #while dist_wp > threshold and wp <= 5:
        y0 = [curr_x, curr_y, curr_si]
        start = time.time()    
        #每次解算一个步长后的推理状态 不一定合理 时间和步长要修改
        sol = solve_ivp(ode_carrot_chase, [0,0.2], y0, method = 'RK23',t_eval=t_eval,args=(w1_x, w1_y, w2_x, w2_y,velocity,delta,k))
        last_theta = None
        last_theta_u = None
        last_si = None
        last_si_d = None
        if self.count % 100 == 0:
            print("solve Time: {:.4f}s".format(time.time()-start))
            print("distance_error: ",dist_wp)
        y = sol.y.T    
        #curr_x, curr_y, curr_si = y[-1, 0], y[-1, 1], y[-1, 2]
        _, _, curr_si = y[-1, 0], y[-1, 1], y[-1, 2]#定速 只需要当前角度即可
        unity_state.drone_speed_control[0] = velocity*np.cos(curr_si)
        unity_state.drone_speed_control[2] = velocity*np.sin(curr_si)
        #dist_wp = np.sqrt((w2_x - curr_x)**2 + (w2_y - curr_y)**2)       
        if dist_wp < threshold and self.waypoints_index < self.waypoints_num-1:
            print(f"waypoint reached: {self.waypoints_index}")
            self.waypoints_index += 1
        if self.waypoints_index == self.waypoints_num-1:
            print("Mission Completed!")
            unity_state.drone_speed_control[0] = 0
            unity_state.drone_speed_control[2] = 0
