import numpy as np
import matplotlib.pyplot as plt
import time
import math
n = 300000
# distance
dis21 = np.zeros((n,1))
dis31 = np.zeros((n,1))
dis41 = np.zeros((n,1))
dis32 = np.zeros((n,1))
dis42 = np.zeros((n,1))
# velocity
v1= np.zeros((n,2))
v2= np.zeros((n,2))
v3= np.zeros((n,2))
v4= np.zeros((n,2))
# position
p1= np.zeros((n,2))
p2= np.zeros((n,2))
p3= np.zeros((n,2))
p4= np.zeros((n,2))

# set distance
d21=3
d31=4
d32=5
d41=5
d42=4

# clockwise or counter clockwise
gama3 = -1
gama4 = 1

pic_num = 1

# dt
dt = 0.01


def init():
    # position init
    p1[0,:]=[0,0]
    p2[0,:]=[6,0]
    p3[0,:]=[3,4]
    p4[0,:]=[0,4]
    # velocity init
    v1[0,:]=[0,0]
    v2[0,:]=[0,0]
    v3[0,:]=[0,0]
    v4[0,:]=[0,0]

def update():
    for i in range(0,n-1):
        t = dt * i
        # update distance
        dis21[i] = dis(p1[i,:], p2[i,:])
        dis31[i] = dis(p1[i,:], p3[i,:])
        dis41[i] = dis(p1[i,:], p4[i,:])
        dis32[i] = dis(p3[i,:], p2[i,:])
        dis42[i] = dis(p4[i,:], p2[i,:])
        # update velocity
        # v1
        v1 = [np.random.uniform(low=0.0, high=5.0, size=None), np.random.uniform(low=0.0, high=1.0, size=None)]
        # v2
        v2 = -(np.square(dis21[i]) -np.square(d21)) * (p2[i,:] - p1[i,:]) 

        # v3
        y1, y2, d1, d2 = transforming(d32, d31, p2[i,:], p1[i,:])
        s3 = cal_s(y1, y2, d1, d2)
        Rs3 = np.array([[s3, gama3 * math.sqrt(1 - s3*s3)],[-gama3 * math.sqrt(1 - s3*s3), s3]])
        A = (p3[i,:] - y1).T
        B = (y1 - y2).T
        v3 = -dis21[i] ** 2 * A - np.dot(d1 * float(dis21[i]) * Rs3, np.array(B))
        # v4
        y3, y4, d3, d4 = transforming(d42, d41, p2[i,:], p1[i,:])
        s4 = cal_s(y3, y4, d3, d4)
        Rs4 = np.array([[s4, gama4 * math.sqrt(1 - s4*s4)],[-gama4 * math.sqrt(1 - s4*s4), s4]])
        C = (p4[i,:] - y3).T
        D = (y3 - y4).T
        v4 = -dis21[i] ** 2 * C - np.dot(d1 * float(dis21[i]) * Rs4, np.array(D))
        # update position
        p1[i+1,0] = p1[i,0] + dt * v1[0]
        p1[i+1,1] = p1[i,1] + dt * v1[1]
        p2[i+1,0] = p2[i,0] + dt * v2[0]
        p2[i+1,1] = p2[i,1] + dt * v2[1]
        p3[i+1,0] = p3[i,0] + dt * v3[0]
        p3[i+1,1] = p3[i,1] + dt * v3[1]
        p4[i+1,0] = p4[i,0] + dt * v4[0]
        p4[i+1,1] = p4[i,1] + dt * v4[1]
        draw_gif(i)

def draw_gif(i):
    if (i % 1 == 0 and i < 20) or (i % 5 == 0 and i > 20 and i < 100) or (i % 10 == 0 and i >100 or i < 200) or (i % 50 ==0 and i > 200):
        plt.clf()
        plt.plot(p2[i,0],p2[i,1],'c.','markersize',30)
        plt.plot(p3[i,0],p3[i,1],'r.','markersize',30)
        plt.plot(p4[i,0],p4[i,1],'g.','markersize',30)
        plt.plot(p1[i,0],p1[i,1],'b.','markersize',30)
        plt.plot([p3[i,0],p2[i,0]],[p3[i,1],p2[i,1]],'b-','linewidth',1.5)
        plt.plot([p3[i,0],p1[i,0]],[p3[i,1],p1[i,1]],'b-','linewidth',1.5)
        plt.plot([p4[i,0],p2[i,0]],[p4[i,1],p2[i,1]],'b-','linewidth',1.5)
        plt.plot([p4[i,0],p1[i,0]],[p4[i,1],p1[i,1]],'b-','linewidth',1.5)
        plt.plot([p1[i,0],p2[i,0]],[p1[i,1],p2[i,1]],'b-','linewidth',1.5)
        plt.text(p1[i,0]-1.5,p1[i,1],'agent1')
        plt.text(p2[i,0]+0.5,p2[i,1],'agent2')
        plt.text(p3[i,0]+0.5,p3[i,1],'agent3')
        plt.text(p4[i,0]+0.5,p4[i,1],'agent4')
        plt.xlim(-2,11)
        plt.ylim(-5,6)
        plt.draw()
        plt.pause(0.001)
    

def dis(p1, p2):
    distance = math.sqrt(np.square(p1[0] - p2[0]) + np.square(p1[1] - p2[1]))
    return distance

def cal_s(y1, y2, d1, d2):
    distance = dis(y1, y2)
    if distance > d1 - d2 and distance < d1 + d2:
        y = 1 + ((distance - (d1 + d2)) * (distance - (d1 -d2))) / (2 * d1 * distance)
    else: y = 1
    return y

def transforming(dx1, dx2, x1, x2):
    if dx1 > dx2:
        y1 = x1
        y2 = x2
        d1 = dx1
        d2 = dx2
    else :
        y1 = x2
        y2 = x1
        d1 = dx2
        d2 = dx1
    return y1, y2, d1, d2

def main():
    init()
    update()


main()