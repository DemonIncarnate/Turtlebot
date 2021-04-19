#! /usr/bin/python

# generate random Gaussian values
import numpy as np
import rospy
import roslaunch
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion

from tf2_msgs.msg import TFMessage
import subprocess

global rx,ry
rx = [6.0, 5.8784054118336195, 5.766300886118646, 5.833599092731074, 5.60039285658449, 5.2988883830557745, 4.926553895308965, 4.659524757102644, 4.667312132498225, 4.334986342517558, 3.8215985780340804, 3.782187604060038, 3.5902660832807154, 3.1828248426456307, 2.8499729614401574, 2.543864487136319, 2.3529652555465717, 1.9559815871473658, 1.6782352509330123, 1.2915590924392548, 0.928728201369079, 0.0]
ry = [6.0, 5.8953995416119085, 5.369865680190552, 4.722197778676184, 4.279975078301957, 4.103689287519464, 4.25911110486447, 3.858278019252686, 3.4585742293627812, 3.0920227157633944, 2.849028999222613, 2.570883060081134, 2.279590807691726, 1.9684323651190339, 1.9823921159557767, 2.0970921787463888, 1.686096599201498, 1.3472570490894413, 0.9643354742411636, 0.8007629165186028, 0.7254658172745165, 0.0]

def odom_callback(odometry):

    global initial_position

    quaternion = tf.transformations.quaternion_from_euler(0,0, initial_position[2])

    position = Pose()

    position.position.x = initial_position[0]
    position.position.y = initial_position[1]

    position.orientation.x = quaternion[0]
    position.orientation.y = quaternion[1]
    position.orientation.z = quaternion[2]
    position.orientation.w = quaternion[3]

    # print("Position is {}".format(position))

    goal_x = initial_position[0]
    goal_y = initial_position[1]

    lin_pos,rot_pos = get_odom()
    velocity = Twist()

    goal_distance = np.sqrt(np.power(goal_x - lin_pos.x, 2) + np.power(goal_y - lin_pos.y, 2))
    distance = goal_distance

    linear_speed = 1

    while distance > 0.05:
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

            velocity.linear.x = control_signal_distance
            velocity.angular.y = control_signal_angle

        vel_pub.publish(velocity)



def get_odom():
    try:
        (trans, rot) = tf_listener.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        rotation = euler_from_quaternion(rot)

    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return
 
    return (Point(*trans), rotation[2])


if __name__ == '__main__':


    

    le = len(rx)
    angle = [2]
    for i in range(le-1):
        initial_position = 0
        coord = [] 
        coord.append(rx[le-1-i])
        coord.append(ry[le-1-i])

        angle[0]=0

        initial_position = np.concatenate((coord,angle))

    
        
        print('(X, Y, Theta):', initial_position[0], initial_position[1], initial_position[2])

        print(initial_position)

        x_final = rx[le-2-i]
    
        y_final = ry[le-2-i]
    
        angle_final = 0

        final = [x_final, y_final, angle_final]
        final_position = np.array(final)
    

    

        #subprocess.run(["obs_world.launch", initial_position[0], initial_position[1], initial_position[2]])

        rospy.init_node('turtlebot3_controller')

        tf_listener = tf.TransformListener()
        vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/tf', TFMessage, odom_callback)

    rospy.spin()