#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Int32,Float64MultiArray,MultiArrayLayout,MultiArrayDimension
import random
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
global arr1,obsx,obsy,path
path= []
arr1= []
obsx= []
obsy = []

N_SAMPLE = 50
N_KNN = 4
MAX_EDGE_LEN= 4




class points:

    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index

def sample_points(start_x, start_y, goal_x, goal_y, robot_size, obsx, obsy):
    max_x = 6.0
    max_y = 6.0
    min_x = 0.0
    min_y = 0.0
    a =True
    sample_x, sample_y = [], []
    oxx = obsx
    oyy = obsy

    while len(sample_x) <= N_SAMPLE:
        tx = (random.random() * (max_x - min_x)) + min_x
        ty = (random.random() * (max_y - min_y)) + min_y
        for i in range(len(oyy)):
            x_ = tx - oxx[i]
            y_ = ty - oyy[i]
            d_ = math.hypot(x_,y_)
            if d_ >= robot_size:
                a =True
                continue
            else:
                a = False
                break

        if a == True:
            sample_x.append(tx)
            sample_y.append(ty)

    sample_x.append(start_x)
    sample_y.append(start_y)
    sample_x.append(goal_x)
    sample_y.append(goal_y)
    #print(sample_y)

    return sample_x, sample_y





def collision(start_x,start_y,goal_x,goal_y,robot_size,obsx,obsy):

    ox1 = obsx
    oy1 = obsy
    
    
    x = start_x
    y = start_y
    dx = goal_x - start_x
    dy = goal_y - start_y
    d = math.hypot(dx,dy)
    for i in range(len(ox1)):
        if d != 0:
            di = abs((dx*(oy1[i]-start_y) - dy*(ox1[i]-start_x))/d)
            if (di <= robot_size):
                return True
        if d == 0:
            return True
    return False
    

def road_map(sample_x,sample_y,robot_size,obsx,obsy):
    road = []
    n = len(sample_x)
    
    sample_kd_tree = cKDTree(np.vstack((sample_x, sample_y)).T)
    for (i, ix, iy) in zip(range(n), sample_x, sample_y):
        dists, indexes = sample_kd_tree.query([ix, iy], k=n)
        edge_id = []

        for ii in range(1, len(indexes)):
            nx = sample_x[indexes[ii]]
            ny = sample_y[indexes[ii]]
            if not collision(ix, iy, nx, ny, robot_size,obsx,obsy):
                edge_id.append(indexes[ii])

            if len(edge_id) >= N_KNN:
                break

        road.append(edge_id)
    #print road
    return road

def Astar(start_x,start_y,goal_x,goal_y,road,sample_x,sample_y):
    start_point = points(start_x,start_y,0.0,-1)
    goal_point = points(goal_x,goal_y,0.0,-1)

    open_set,closed_set = dict(),dict()
    open_set[len(road)- 2] = start_point
    path = True

    while True:
        if not open_set:
            path = False
            break
        current_id = min(open_set,key=lambda o: open_set[o].cost)
        current_point = open_set[current_id]

        if current_id == (len(road)-1):
            goal_point.parent_index = current_point.parent_index
            goal_point.cost = current_point.cost
            break

        del open_set[current_id]
        closed_set[current_id] = current_point
        for i in range(len(road[current_id])):
            next_id = road[current_id][i]
            dx = sample_x[next_id]-current_point.x
            dy = sample_y[next_id]- current_point.y
            hx = abs(sample_x[next_id] - goal_x)
            hy = abs(sample_y[next_id] - goal_y)
            d = math.hypot(dx, dy)
            h = hx + hy
            point   =  points(sample_x[next_id],sample_y[next_id],current_point.cost +d +h ,current_id)

            if next_id in closed_set:
                continue
            if next_id in open_set:
                if open_set[next_id].cost < point.cost:
                    open_set[next_id].cost = point.cost
                    open_set[next_id].parent_index = current_id
            else:
                open_set[next_id] = point
    if path is False:
        return [], []       
    rx, ry = [goal_point.x],[goal_point.y]
    parent_index = goal_point.parent_index
    while parent_index != -1:
        n = closed_set[parent_index]
        rx.append(n.x)
        ry.append(n.y)
        parent_index = n.parent_index
    
    
    return rx, ry


def final(start_x,start_y,goal_x,goal_y,obsx,obsy,robot_size):
    sample_x ,sample_y = sample_points(start_x,start_y,goal_x,goal_y,robot_size,obsx,obsy)
    
    road = road_map(sample_x,sample_y,robot_size,obsx,obsy)

    rx,ry = Astar(start_x,start_y,goal_x,goal_y,road,sample_x,sample_y)

    return rx,ry

def main1():
    print(__file__ + " start!!")

   
    start_x = 0.0  
    start_y = 0.0  
    goal_x = 6.0  
    goal_y = 6.0  
    robot_size = 0.45 

    rx,ry = final(start_x,start_y,goal_x,goal_y,obsx,obsy,robot_size)

    path  = np.concatenate((rx,ry))
    print(path)
    return path

    
class planner():
    def __init__(self):

        rospy.init_node('planner', anonymous=True)
        self.pub = rospy.Publisher('planner',Float64MultiArray,queue_size=10)
        self.sub =rospy.Subscriber("obstacles_pub",Float64MultiArray , self.callback)
        self.rate = rospy.Rate(50) # 50 Hz
        
        self.data2 = Float64MultiArray()
        self.data2.data =  main1()



    def publish(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.data2)
            self.rate.sleep
        

    def callback(self,data):
        arr1 = data.data
        for i in range(0,14):
            obsx.append(arr1[2*i])
            obsy.append(arr1[2*i+1])
        #print(arr1)
    


if __name__ == '__main__':
    
    o = planner()
    o.publish()