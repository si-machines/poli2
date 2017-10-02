#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

"""
Teleop node for a playstation 3 controller.
In order to first connect the controller, install sixad with apt.
Then, with the controller connected via usb, run sudo sixpair.
Finally, to connect the controller going forward, run sixad --start.

This node uses the directional pad for movement. The back right trigger (R2) is
a deadman switch, which must be pressed for any movement.

Author: Max Svetlik

MODES of operation:
CROSS = base movement
CIRCLE = gripper movement
SQUARE = head movement

"""


# 13 = back right trigger = dead man switch
# 8 - UP
# 10 - DOWN
# 9 - RIGHT
# 11 - LEFT

def callback(data):
    global base_pub, pan_pub, tilt_pub, gripper_pub, last_gripper, alpha, mode

    if data.axes[18] != 0: # 'X'
        mode = "base"
    elif data.axes[19] != 0: # 'SQUARE'
        mode = "head"
    elif data.axes[17] != 0: # 'O'
        mode = "gripper"

    twist = Twist()
    alpha = 1.0  # linear scale factor on the lower actuator of logitech controler
    beta = 1.0  # angular scale factor
    if data.axes[13] < -0:
        if mode is "base":
            twist.linear.x = alpha*(-data.axes[8] + data.axes[10])
            twist.angular.z = beta*(-data.axes[11] + data.axes[9])
            base_pub.publish(twist)
        elif mode is "head":
            tilt = Float64()
            pan = Float64()
            tilt = (data.axes[8] - data.axes[10])
            pan = (-data.axes[11] + data.axes[9])
            tilt_pub.publish(tilt)
            pan_pub.publish(pan)

def start():
    global base_pub, pan_pub, tilt_pub, gripper_pub, mode, last_gripper
    mode = "base"
    rospy.init_node('ps_teleop_node', anonymous=True)
    base_pub = rospy.Publisher('/poli/teleop/cmd_vel', Twist, queue_size=5)
    pan_pub = rospy.Publisher('/pan_controller/command', Float64, queue_size=1)
    tilt_pub = rospy.Publisher('/tilt_controller/command', Float64, queue_size=1)

    rospy.Subscriber('/ps/joy', Joy, callback)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    start()

