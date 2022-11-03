#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# LIN_STOP_VAL = 0.1 # m/s
# ANG_STOP_VAL = 0.1 # deg/s

def cmd_vel_callback(data):
    global dlp_pub
    global last_cmd

    global SPEED_LINEAR_FAST
    global SPEED_LINEAR_SLOW
    global SPEED_LINEAR_STOP

    linear_vel = data.linear.x
    angular_vel = data.angular.z

    # if abs(linear_vel) < LIN_STOP_VAL and abs(angular_vel) < ANG_STOP_VAL:
    #     cmd = "stop"
    # elif angular_vel <= -0.33:
    #     cmd = "turn_left"
    # elif angular_vel >= 0.33:
    #     cmd = "turn_right"
    # else:
    #     cmd = "go"

    print("dlp_interface: SPEED_LINEAR_FAST = {}".format(SPEED_LINEAR_FAST))

    speed_margin = (SPEED_LINEAR_FAST - SPEED_LINEAR_SLOW)/2.0
    if linear_vel <= SPEED_LINEAR_STOP:
        cmd = "stop"
    elif linear_vel < SPEED_LINEAR_SLOW + speed_margin:
        cmd = "slow"
    else:
        cmd = "go"

    if cmd != last_cmd:
        dlp_pub.publish(cmd)
        last_cmd = cmd

def dlp_interface():
    rospy.init_node('dlp_interface', anonymous=True)

    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)

    global SPEED_LINEAR_FAST
    global SPEED_LINEAR_SLOW
    global SPEED_LINEAR_STOP

    SPEED_LINEAR_FAST = rospy.get_param("~speed_linear_fast", 0.8)
    SPEED_LINEAR_SLOW = rospy.get_param("~speed_linear_slow", 0.4)
    SPEED_LINEAR_STOP = rospy.get_param("~speed_linear_stop", 0.0)

    global dlp_pub
    dlp_pub = rospy.Publisher('/dlp_test_string', String, queue_size=2)

    global last_cmd
    last_cmd = "stop"
    dlp_pub.publish(last_cmd)

    rospy.spin()


if __name__ == '__main__':
    try:
        dlp_interface()
    except rospy.ROSInterruptException:
        pass
