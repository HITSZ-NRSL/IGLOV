#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import sys, select, termios, tty

msg = """
Control The Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
        'w':(1,0),
        'e':(1,-1),
        'a':(0,1),
        'd':(0,-1),
        'q':(1,1),
        'x':(-1,0),
        'c':(-1,1),
        'z':(-1,-1),
           }

speedBindings={
        'u':(1.1,1.1),
        'j':(.9,.9),
        'i':(1.1,1),
        'k':(.9,1),
        'o':(1,1.1),
        'l':(1,.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

global_position = [0,0,0,0,0,0,0]

def Position_callback(odom_data):
    curr_time = odom_data.header.stamp
    global_position[0] = odom_data.pose.pose.position.x #  the x,y,z pose and quaternion orientation
    global_position[1] = odom_data.pose.pose.position.y
    global_position[2] = odom_data.pose.pose.position.z
    global_position[3] = odom_data.pose.pose.orientation.x #  the x,y,z pose and quaternion orientation
    global_position[4] = odom_data.pose.pose.orientation.y
    global_position[5] = odom_data.pose.pose.orientation.z
    global_position[6] = odom_data.pose.pose.orientation.w


count_stage = 0
last_position = 0

##################################################################################
# set_position_outnone = [-1.1, 4.0, -6, -4, 4.0, 4, 6, -4, -0.5]     # 沿边线行走，增量
# set_position_outnone = [-4.00, 4.50, -16.00, -3.5, -8.00, 4.50, 4.00, -3.5, 2.5]     # 沿边线行走-名义-10

# set_position_outnone = [-1.1, 2.6, -9.0, -1.6, -5.00, 2.6, 3.0, -1.6, 2.5]     # 沿边线行走-实际
# set_position_outnone = [-0.9, 2.4, -8.8, -1.4, -5.2, 2.4, 2.8, -1.4, 2.5]     # 沿边线行走 -0.2
# set_position_outnone = [-0.7, 2.2, -8.6, -1.2, -5.4, 2.2, 2.6, -1.2, 2.5]     # 沿边线行走 -0.4
# set_position_outnone = [-0.5, 2.0, -8.4, -1.0, -5.6, 2.0, 2.4, -1.0, 1.9]     # 沿边线行走 -0.6
# set_position_outnone = [-0.3, 1.8, -8.2, -0.8, -5.8, 1.8, 2.2, -0.8, 1.7]     # 沿边线行走 -0.8
# set_position_outnone = [-0.1, 1.6, -8.0, -0.6, -6.0, 1.6, 2.0, -0.6, 1.5]     # 沿边线行走 -1.0
# set_position_outnone = [0.1, 1.4, -7.8, -0.4, -6.2, 1.4, 1.8, -0.4, 1.3]     # 沿边线行走 -1.2

##################################################################################################
######记得同时切换gazebo.launch里的位置
# set_position_outnone = [-3.00, 3.50, -15.00, -2.5, -9.00, 3.50, 3.00, -2.5, 2.5]     # 沿边线行走-名义
# set_position_outnone = [-2.5, 3.0, -14.5, -2.0, -9.5, 3.0, 2.5, -2.0, 2.5]     # 沿边线行走-实际
# set_position_outnone = [-(2.5+dis), 3.0+dis, -(14.5+dis), -(2.0+dis), -(9.5-dis), 3.0+dis, 2.5+dis, -(2.0+dis), 2+dis]     # 沿边线行走-delta
# set_position_outnone = [(4-dis), 3.0+dis, -(9+dis), -(2.0+dis), -(4.1-dis), 3.0+dis, 8.8+dis, -(2.0+dis), 7+dis]    # grass_water_terrain 0.04 0.006

dis = -1.0   # +:特征更丰富            -：向featureless运动6
set_position_outnone = [(4.6-dis), 2.0+dis, -(8.4+dis), -(1.0+dis), -(5.1-dis), 2.0+dis, 8.1+dis, -(1.0+dis), 7+dis]    # grass_water_terrain

# grass_water_terrain_random
# dis = 0.6   # +:特征更丰富            -：向featureless运动6
# set_position_outnone = [(4.6-dis), -(2.0+dis), -(6.4+dis), -(3.6+dis), -(3.1-dis), -(2.0+dis), 6.1+dis, -(3.6+dis), 5+dis]    

set_rotation_outnone = [-0.695, -0.017, 0.695, -0.01, 0.695, 0.01, -0.69, 0.01]     #genral scene

# speed = .015
# turn = .036
speed = .04
turn = .04
# speed = .03
# turn = .045
# speed = .045
# turn = .108

def run_loop():
    global speed
    global turn
    print "run_loop"
    print(global_position)
    
    global count_stage
    global last_position
    # print count_stage
    if global_position[0] > set_position_outnone[0] and count_stage == 0:
        return 'w'
    elif  count_stage == 0:
        last_position = global_position[1]
        count_stage = 1
        # speed = .03
    
    # print count_stage
    if count_stage == 1 and global_position[6] > set_rotation_outnone[0]:
        print('2e:',count_stage)
        print('last_position:',last_position)
        return 'e'
    elif count_stage == 1:
        print('2e')
        last_position = global_position[1]
        count_stage = 2
        # speed = .03
    
    # print count_stage
    if count_stage == 2 and global_position[1] < (set_position_outnone[1]):
        print('3w:',count_stage)
        print('last_position:',last_position)
        return 'w'
    elif count_stage == 2:
        print('3w')
        last_position = global_position[1]
        count_stage = 3
        # speed = .03
    
    # print count_stage
    if count_stage == 3 and global_position[6] < set_rotation_outnone[1]:
        print('4q:',count_stage)
        return 'q'
    elif count_stage == 3:
        last_position = global_position[0]
        count_stage = 4
        # speed = .03
    
    # print count_stage
    if count_stage == 4 and global_position[0] > (set_position_outnone[2]):
        print('5w:',count_stage)
        print('last_position:',last_position)
        return 'w'
    elif count_stage == 4:
        last_position = global_position[1]
        count_stage = 5
        # speed = .03
    
    # print count_stage
    if count_stage == 5 and global_position[6] < set_rotation_outnone[2]:
        print('6q', count_stage)
        print('last_position:',last_position)
        return 'q'
    elif count_stage == 5:
        last_position = global_position[1]
        count_stage = 6
        # speed = .03

    if count_stage == 6 and global_position[1] > (set_position_outnone[3]):
        print('7w', count_stage)
        print('last_position:',last_position)
        return 'w'
    elif count_stage == 6:
        last_position = global_position[0]
        count_stage = 7
        # speed = .03

    # print count_stage
    if count_stage == 7 and global_position[5] < set_rotation_outnone[3]:
        print('8q', count_stage)
        return 'q'
    elif count_stage == 7:
        last_position = global_position[0]
        count_stage = 8
        # speed = .03

    # print count_stage
    if count_stage == 8 and global_position[0] < (set_position_outnone[4]):
        print('9w', count_stage)
        return 'w'
    elif count_stage == 8:
        last_position = global_position[0]
        count_stage = 9
        # speed = .03

    # print count_stage
    if count_stage == 9 and global_position[5] < set_rotation_outnone[4]:
        print('10q', count_stage)
        return 'q'
    elif count_stage == 9:
        last_position = global_position[1]
        count_stage = 10
        # speed = .03

    # print count_stage
    if count_stage == 10 and global_position[1] < (set_position_outnone[5]):
        print('11w', count_stage)
        return 'w'
    elif count_stage == 10:
        last_position = global_position[0]
        count_stage = 11
        # speed = .03
    
    # print count_stage
    if count_stage == 11 and global_position[5] > set_rotation_outnone[5]:
        print('12e', count_stage)
        return 'e'
    elif count_stage == 11:
        last_position = global_position[0]
        count_stage = 12
        # speed = .03

    # print count_stage
    if count_stage == 12 and global_position[0] < (set_position_outnone[6]):
        print('13w', count_stage)
        return 'w'
    elif count_stage == 12:
        last_position = global_position[0]
        count_stage = 13
        # speed = .03

    # print count_stage
    if count_stage == 13 and global_position[5] > set_rotation_outnone[6]:
        print('14e', count_stage)
        return 'e'
    elif count_stage == 13:
        last_position = global_position[1]
        count_stage = 14
        # speed = .03

    # print count_stage
    if count_stage == 14 and global_position[1] > (set_position_outnone[7]):
        print('15w', count_stage)
        return 'w'
    elif count_stage == 14:
        last_position = global_position[0]
        count_stage = 15
        # speed = .03
    
    # print count_stage
    if count_stage == 15 and global_position[6] > set_rotation_outnone[7]:
        print('16e', count_stage)
        return 'e'
    elif count_stage == 15:
        last_position = global_position[0]
        count_stage = 16
        # speed = .03

    # print count_stage
    if count_stage == 16 and global_position[0] > (set_position_outnone[8]):
        print('17w', count_stage)
        return 'w'
    elif count_stage == 16:
        global runloop_flag
        runloop_flag = 0
        count_stage = 0


runloop_flag = 0
if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('robot_teleop')
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
    rospy.Subscriber("/ground_truth/feedback",Odometry,Position_callback) 

    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print msg
        print vels(speed,turn)
        while(1):
            key = getKey()
            if runloop_flag == 1:
                if (key == '\x03'):
                    print "exit"
                    break
                if (key!='s'):
                  key = run_loop()
                  print key
                
        
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0

                print vels(speed,turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            elif key == 's' :#key == ' ' or
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            elif key == 'r':
                print "Running Loop"
                runloop_flag = 1
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    print "exit"
                    break

            target_speed = speed * x
            target_turn = turn * th

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.02 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.02 )
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.1 )
            else:
                control_turn = target_turn

            twist = Twist()
            twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            pub.publish(twist)

    except:
        print "error"

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
