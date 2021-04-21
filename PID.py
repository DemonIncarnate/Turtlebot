#!/usr/bin/env python


#rx = [6.0, 5.8784054118336195, 5.766300886118646, 5.833599092731074, 5.60039285658449, 5.2988883830557745, 4.926553895308965, 4.659524757102644, 4.667312132498225, 4.334986342517558, 3.8215985780340804, 3.782187604060038, 3.5902660832807154, 3.1828248426456307, 2.8499729614401574, 2.543864487136319, 2.3529652555465717, 1.9559815871473658, 1.6782352509330123, 1.2915590924392548, 0.928728201369079, 0.0]
#ry = [6.0, 5.8953995416119085, 5.369865680190552, 4.722197778676184, 4.279975078301957, 4.103689287519464, 4.25911110486447, 3.858278019252686, 3.4585742293627812, 3.0920227157633944, 2.849028999222613, 2.570883060081134, 2.279590807691726, 1.9684323651190339, 1.9823921159557767, 2.0970921787463888, 1.686096599201498, 1.3472570490894413, 0.9643354742411636, 0.8007629165186028, 0.7254658172745165, 0.0]


import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
import os
import time
import math 
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from tf.transformations import quaternion_from_euler

global arr1,rx,ry,x_input,y_input,z_input
arr1 = []
rx = []
ry = []
x_input = 0
y_input= 0
z_input = 0
kp_distance = 1
ki_distance = 0.01
kd_distance = 0.7

kp_angle = 1
ki_angle = 0.03
kd_angle = 0.07

class GotoPoint():
    def __init__(self):
        rospy.init_node('turtlebot3_pointop_key', anonymous=True)
        self.sub = rospy.Subscriber('planner',Float64MultiArray,self.path)
       
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(50)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'

        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                
                rospy.signal_shutdown("tf Exception")

        (position, rotation) = self.get_odom()


        last_rotation = 0
        linear_speed = 1   
        angular_speed = 1  


        (goal_x, goal_y, goal_z) = self.variable()

        goal_z = np.deg2rad(goal_z)
 
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        
        distance = goal_distance
        previous_distance = 0
        total_distance = 0

        previous_angle = 0
        total_angle = 0
       

        while distance > 0.05:
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
            
            path_angle = atan2(goal_y - y_start, goal_x- x_start)

            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2*pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2*pi + path_angle
            if last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation


            diff_angle = path_angle - previous_angle
            diff_distance = distance - previous_distance

            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))

            control_signal_distance = kp_distance*distance + ki_distance*total_distance + kd_distance*diff_distance

            control_signal_angle = kp_angle*path_angle + ki_angle*total_angle + kd_distance*diff_angle

            move_cmd.angular.z = (control_signal_angle) - rotation
            
            move_cmd.linear.x = min(control_signal_distance, 0.1)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            last_rotation = rotation
            self.cmd_vel.publish(move_cmd)
            r.sleep()
            previous_distance = distance
            total_distance = total_distance + distance
            print("Current position and rotation are: ", (position, rotation))

        (position, rotation) = self.get_odom()
        print("Current position and rotation are: ", (position, rotation))

        print("reached")

        while abs(rotation - goal_z) > 0.05:
            (position, rotation) = self.get_odom()
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
            self.cmd_vel.publish(move_cmd)
            


        
        self.cmd_vel.publish(Twist())
        return

    def path(self,data):
        arr1 = data.data
        a = len(arr1)
        b = a/2
        for i in range(0,b):
            rx.append(arr1[i])
            ry.append(arr1[b+i])
        #print(rx)
    
    
    
    def variable(self):
        global x_input, y_input, z_input
        x = x_input
        y = y_input
        z = z_input
        x, y, z = [float(x), float(y), float(z)]
        return x, y, z

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return

        return (Point(*trans), rotation[2])


    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


def main1(): 
    initial_position= []
    coord = []
    k = 0
    r = 0
    coord.append(k)
    coord.append(r)
    angle2=[1]
    angle2[0]= 0
    initial_position = np.concatenate((coord,angle2))
    x_final = 0
    
    y_final = 0
    
    angle_final = 0

    final = [x_final, y_final, angle_final]
    final_position = np.array(final)

    x_input = final_position[0]
    y_input = final_position[1]
    z_input = final_position[2]


    q = quaternion_from_euler(0, 0, initial_position[2])
       
    state_msg = ModelState()
    state_msg.model_name = 'turtlebot3_burger'
    state_msg.pose.position.x = initial_position[0]
    state_msg.pose.position.y = initial_position[1]
    state_msg.pose.position.z = 0

    state_msg.pose.orientation.x = q[0]
    state_msg.pose.orientation.y = q[1]
    state_msg.pose.orientation.z = q[2]
    state_msg.pose.orientation.w = q[3]

    time.sleep(5)
    GotoPoint()
 
#if __name__ == '__main__':

main1()
     
le =  len(rx)
angle= [2]
for i in range(le -1):
    initial_position = []
    coord = [] 
    coord.append(rx[le-1-i])
    coord.append(ry[le-1-i])

    angle[0]=0

    initial_position = np.concatenate((coord,angle))
    print(initial_position)

    x_final = rx[le-2-i]
    
    y_final = ry[le-2-i]
    
    angle_final = math.atan2(y_final-coord[1],x_final-coord[0])

    final = [x_final, y_final, angle_final]
    final_position = np.array(final)

    x_input = final_position[0]
    y_input = final_position[1]
    z_input = final_position[2]


    q = quaternion_from_euler(0, 0, initial_position[2])
       
    state_msg = ModelState()
    state_msg.model_name = 'turtlebot3_burger'
    state_msg.pose.position.x = initial_position[0]
    state_msg.pose.position.y = initial_position[1]
    state_msg.pose.position.z = 0

    state_msg.pose.orientation.x = q[0]
    state_msg.pose.orientation.y = q[1]
    state_msg.pose.orientation.z = q[2]
    state_msg.pose.orientation.w = q[3]

 

    time.sleep(5)

        
    GotoPoint()
