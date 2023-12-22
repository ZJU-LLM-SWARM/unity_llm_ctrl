#! /usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import time
from .cvxpy_mpc.utils import compute_path_from_wp, get_ref_trajectory
from .cvxpy_mpc import MPC, VehicleModel
import sys
import json

class MPCfollow:
    def __init__(self):

        with open("Assets/LLM_ENV/json_data/waypoint_settings.json", "r") as f:
            data = json.load(f)

        SIM_START_X = data["SIM_START_X"]
        SIM_START_Y = data["SIM_START_Y"]
        SIM_START_V = data["SIM_START_V"]
        SIM_START_H = data["SIM_START_H"]

        self.TARGET_VEL = data["TARGET_VEL"]
        self.T = data["Prediction_Horizon"]
        self.DT = data["Discretization_step"]
        self.L = data["vehicle_wheelbase"]
        Q = [20, 20, 10, 20]  # state error cost
        Qf = [30, 30, 30, 30]  # state final error cost
        R = [10, 10]  # input cost
        P = [20, 20]  # input rate of change cost

        # State of the robot [x,y,v, heading]
        self.state = np.array([SIM_START_X, SIM_START_Y, SIM_START_V, SIM_START_H])
        # helper variable to keep track of mpc output
        # starting condition is 0,0
        self.control = np.zeros(2)
        

        self.K = int(self.T / self.DT)
        self.mpc = MPC(VehicleModel(), self.T, self.DT, Q, Qf, R, P)

        # Path from waypoint interpolation
        waypoints_x = data["waypoints_x"]
        waypoints_y = data["waypoints_y"]
        interpolation_interval = data["interpolation_interval"]
        self.path = compute_path_from_wp(
            waypoints_x,
            waypoints_y,
            interpolation_interval,
        )

        # Helper variables to keep track of the sim
        self.sim_time = 0
        self.x_history = []
        self.y_history = []
        self.v_history = []
        self.h_history = []
        self.a_history = []
        self.d_history = []
        self.optimized_trajectory = None

        # Initialise plot
        # plt.style.use("ggplot")s
        # self.fig = plt.figure()
        # plt.ion()
        # plt.show()

    def ego_to_global(self, mpc_out):
        """
        transforms optimized trajectory XY points from ego(car) reference
        into global(map) frame

        Args:
            mpc_out ():
        """
        trajectory = np.zeros((2, self.K))
        trajectory[:, :] = mpc_out[0:2, 1:]
        Rotm = np.array(
            [
                [np.cos(self.state[3]), np.sin(self.state[3])],
                [-np.sin(self.state[3]), np.cos(self.state[3])],
            ]
        )
        trajectory = (trajectory.T.dot(Rotm)).T
        trajectory[0, :] += self.state[0]
        trajectory[1, :] += self.state[1]
        return trajectory

    def run(self,unity_state):
        '''
        unity_state: 只传一个agent
        '''
        #self.plot_sim()
        #input("Press Enter to continue...")

        # self.state[0] = unity_state.position[0]
        # self.state[1] = unity_state.position[2]

        # self.state[2] = np.sqrt(unity_state.velocity[0]**2+unity_state.velocity[2]**2)#xz
        # self.state[3] = np.arctan2(unity_state.velocity[2],unity_state.velocity[0])


        #px,py,v,h
        if (
            np.sqrt(
                (self.state[0] - self.path[0, -1]) ** 2
                + (self.state[1] - self.path[1, -1]) ** 2
            )
            < 0.5
        ):
            print("Success! Goal Reached")
            #input("Press Enter to continue...")
            # return
        # optimization loop
        #start=time.time()
        # Get Reference_traj -> inputs are in worldframe

        target = get_ref_trajectory(self.state, self.path, self.TARGET_VEL, self.T, self.DT)


        # dynamycs w.r.t robot frame
        curr_state = np.array([0, 0, self.state[2], 0])
        #delay not stable
        # curr_state[0] = curr_state[0] + curr_state[2] * np.cos(curr_state[3]) * self.DT
        # curr_state[1] = curr_state[1] + curr_state[2] * np.sin(curr_state[3]) * self.DT#速度推理
        # curr_state[2] = curr_state[2] + self.control[0] * self.DT
        # curr_state[3] = curr_state[3] + self.control[0] * np.tan(self.control[1]) / self.L * self.DT#控制输入
        x_mpc, u_mpc = self.mpc.step(
            curr_state,
            target,
            self.control,
            verbose=True,
        )
        #print("CVXPY Optimization Time: {:.4f}s".format(time.time()-start))
        # only the first one is used to advance the simulation
        self.control[:] = [u_mpc.value[0, 0], u_mpc.value[1, 0]]

        vel = self.control[0]*self.DT
        gamma = self.control[1]*self.DT#变化的角度

        # theta_unity = np.arctan2(unity_state.velocity[2],unity_state.velocity[0])
        # theta_unity = theta_unity + np.tan(gamma)/self.L
        #vxvz
        # unity_state.drone_speed_control[0] = unity_state.velocity[0] + vel*np.cos(theta_unity)
        # unity_state.drone_speed_control[2] = unity_state.velocity[2] + vel*np.sin(theta_unity)
        unity_state.drone_speed_control[0] = self.state[2]*np.cos(self.state[3])
        unity_state.drone_speed_control[2] = self.state[2]*np.sin(self.state[3])

        self.state = self.predict_next_state(
                self.state, [self.control[0], self.control[1]], self.DT
            )

        # curr_state[0] = curr_state[0] + curr_state[2] * np.cos(curr_state[3]) * self.DT
        # curr_state[1] = curr_state[1] + curr_state[2] * np.sin(curr_state[3]) * self.DT#速度推理
        # curr_state[2] = curr_state[2] + self.control[0] * self.DT
        # curr_state[3] = curr_state[3] + self.control[0] * np.tan(self.control[1]) / self.L * self.DT#控制输入

        #return self.control#加速度 角速度
        # use the optimizer output to preview the predicted state trajectory
        self.optimized_trajectory = self.ego_to_global(x_mpc.value)#仅画图
        #self.plot_sim()

    def predict_next_state(self, state, u, dt):
        def kinematics_model(x, t, u):
            dxdt = x[2] * np.cos(x[3])
            dydt = x[2] * np.sin(x[3])
            dvdt = u[0]
            dthetadt = x[2] * np.tan(u[1]) / self.L
            dqdt = [dxdt, dydt, dvdt, dthetadt]
            return dqdt

        # solve ODE
        
        tspan = [0, dt]
        new_state = odeint(kinematics_model, state, tspan, args=(u[:],))[1]
        return new_state

    def plot_sim(self):
        self.sim_time = self.sim_time + self.DT
        self.x_history.append(self.state[0])
        self.y_history.append(self.state[1])
        self.v_history.append(self.state[2])
        self.h_history.append(self.state[3])
        self.a_history.append(self.control[0])
        self.d_history.append(self.control[1])

        plt.clf()

        grid = plt.GridSpec(2, 3)

        plt.subplot(grid[0:2, 0:2])
        plt.title(
            "MPC Simulation \n" + "Simulation elapsed time {}s".format(self.sim_time)
        )

        plt.plot(
            self.path[0, :],
            self.path[1, :],
            c="tab:orange",
            marker=".",
            label="reference track",
        )

        plt.plot(
            self.x_history,
            self.y_history,
            c="tab:blue",
            marker=".",
            alpha=0.5,
            label="vehicle trajectory",
        )

        if self.optimized_trajectory is not None:
            plt.plot(
                self.optimized_trajectory[0, :],
                self.optimized_trajectory[1, :],
                c="tab:green",
                marker="+",
                alpha=0.5,
                label="mpc opt trajectory",
            )

        plot_car(self.x_history[-1], self.y_history[-1], self.h_history[-1])

        plt.ylabel("map y")
        plt.yticks(
            np.arange(min(self.path[1, :]) - 1.0, max(self.path[1, :] + 1.0) + 1, 1.0)
        )
        plt.xlabel("map x")
        plt.xticks(
            np.arange(min(self.path[0, :]) - 1.0, max(self.path[0, :] + 1.0) + 1, 1.0)
        )
        plt.axis("equal")
        # plt.legend()

        plt.subplot(grid[0, 2])
        # plt.title("Linear Velocity {} m/s".format(self.v_history[-1]))
        plt.plot(self.a_history, c="tab:orange")
        locs, _ = plt.xticks()
        plt.xticks(locs[1:], locs[1:] * self.DT)
        plt.ylabel("a(t) [m/ss]")
        plt.xlabel("t [s]")

        plt.subplot(grid[1, 2])
        # plt.title("Angular Velocity {} m/s".format(self.w_history[-1]))
        plt.plot(np.degrees(self.d_history), c="tab:orange")
        plt.ylabel("gamma(t) [deg]")
        locs, _ = plt.xticks()
        plt.xticks(locs[1:], locs[1:] * self.DT)
        plt.xlabel("t [s]")

        plt.tight_layout()

        plt.draw()
        plt.pause(0.01)


def plot_car(x, y, yaw):
    """

    Args:
        x ():
        y ():
        yaw ():
    """
    LENGTH = 0.5  # [m]
    WIDTH = 0.25  # [m]
    OFFSET = LENGTH  # [m]

    outline = np.array(
        [
            [-OFFSET, (LENGTH - OFFSET), (LENGTH - OFFSET), -OFFSET, -OFFSET],
            [WIDTH / 2, WIDTH / 2, -WIDTH / 2, -WIDTH / 2, WIDTH / 2],
        ]
    )

    Rotm = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])

    outline = (outline.T.dot(Rotm)).T

    outline[0, :] += x
    outline[1, :] += y

    plt.plot(
        np.array(outline[0, :]).flatten(), np.array(outline[1, :]).flatten(), "tab:blue"
    )


# def do_sim():
#     controller = MPCfollow()
#     try:
#         controller.run()
#     except Exception as e:
#         sys.exit(e)


# if __name__ == "__main__":
#     do_sim()
