#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import time
import PyKDL
import math
from math import pi
from math import floor
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion,quaternion_from_euler 
import sys, select, termios, tty

velocity=[0,0,0,0,0,0]
position=[0,0,0,0,0,0]
gimbal_vec = [0,0,0]
gimbal_pos = [0,0,0]
gimbal_des = [0,0,0]
gimbal_vec_ref = [0,0,0]

# i_clamp_max = 0.5
# out_clamp_max = 1000
Iout = 0

# ref feedback; set 
def PID_calc(Kp, Ki, fed_val, set_val, i_clamp_max, out_clamp_max):
    error = set_val - fed_val
    Pout = Kp * error
    global Iout
    Iout = Iout + Ki * error
    if(Iout > i_clamp_max):
        Iout = i_clamp_max
    elif(Iout < -i_clamp_max):
        Iout = -i_clamp_max
    out = Pout + Iout
    if(out > out_clamp_max):
        out = out_clamp_max
    elif(out < -out_clamp_max):
        out = -out_clamp_max
    return out

def callback(data):
    for i in range(2):
        gimbal_vec[i] = data.velocity[i]
        gimbal_pos[i] = data.position[i]
    # gimbal_pos[0] = gimbal_pos[0] + 0.77        # 此参数影响ORBslam z轴漂移
    
    gimbal_vec_ref[0] = PID_calc(0.6, 0, gimbal_pos[0], gimbal_des[1], 0.2, 0.2)
    gimbal_vec_ref[1] = PID_calc(1, 0, gimbal_pos[1], gimbal_des[2], 0.2, 2)

    pub_p.publish(gimbal_vec_ref[0])
    pub_y.publish(gimbal_vec_ref[1])


# data.x->roll
# data.y->pitch
# data.z->yaw
def callback_bestview(data):
    print("data: %f %f %f \r" % (data.x, data.y, data.z))

    gimbal_des[0] = data.x
    gimbal_des[1] = -data.y
    gimbal_des[2] = data.z

    # Pitch
    if gimbal_des[1] < -math.pi/6:
      gimbal_des[1] = -math.pi/6
    elif gimbal_des[1] > math.pi/6:
      gimbal_des[1] = math.pi/6

    print("gimbal_des: %f %f %f \r" % (gimbal_des[0], gimbal_des[1], gimbal_des[2]))

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__=="__main__":
    rospy.init_node('control')
    settings = termios.tcgetattr(sys.stdin)

    pub_p = rospy.Publisher('/joint10_velocity_controller/command',Float64, queue_size=3)
    pub_y = rospy.Publisher('/joint11_velocity_controller/command',Float64, queue_size=3)

    rospy.Subscriber("/best_view_robot", Vector3, callback_bestview)
    rospy.Subscriber("/joint_states",JointState,callback)

    init_flag = 0

    time.sleep(0.1)
    while(1):
        stop_key = getKey()

        if stop_key == '\x03':
            print "exit"
            break
        


