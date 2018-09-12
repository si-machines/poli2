#!/usr/bin/env python
"""
This program performs initialization behaviors that are non-essential for the components' performance.
These behaviors are merely aesthetic and give an impression that everything is working correctly.

The way the script is written assumes that this will be called directly after robot-start proper.
Specifically, it assumes that the tilt motor will have just been initialized at its hard stop and be at 
the 0.0 position. Is also assumes the pillar will be initialized at its hard stop and be at its 0.0 position.

POST: These assumptions are toggleable

Author Maxwell J Svetlik 

"""

import sys
import rospy
from std_msgs.msg import Float64, Float32
from sensor_msgs.msg import JointState
from poli_msgs.srv import LedEye, LedEar

init_param = "/post_initialization_start/has_initialized"

class PostInitializationStartup:
    def __init__(self):
        self.pillar_pub = rospy.Publisher('/pillar/command', Float32, queue_size=1)
        self.tilt_pub   = rospy.Publisher('/tilt_motor/position_controller/command', Float64, queue_size=1)
        self.pan_pub    = rospy.Publisher('/pan_motor/position_controller/command', Float64, queue_size=1)
        self.pillar_js  = JointState()
        self.tilt_js    = JointState()
        rospy.Subscriber("/pillar/joint_states", JointState, self.pillar_callback)
        rospy.Subscriber("/tilt_motor/joint_states", JointState, self.tilt_callback)

        rospy.wait_for_service("/led_ear")
        rospy.wait_for_service("/led_eye")

        self.led_ear = rospy.ServiceProxy('/led_ear', LedEar)
        self.led_eye = rospy.ServiceProxy('/led_eye', LedEye)

        #Change to TRUE to enforce zero-position assumptions about pillar and tilt motor
        self.check_init_pos = False

    def pillar_callback(self,data):
        self.pillar_js = data

    def tilt_callback(self,data):
        self.tilt_js = data


    def move_pan(self):
        msg = Float64()
        msg.data = 0.4
        self.pan_pub.publish(msg)
        #wait for motion
        rospy.sleep(2.0)
        msg.data = 0.0
        self.pan_pub.publish(msg)

    def move_tilt(self):
        if self.check_init_pos:
            if len(self.tilt_js.position) > 0:
                if abs(self.tilt_js.position[0]) < 0.01:
                    msg = Float64()
                    msg.data = -0.5
                    self.tilt_pub.publish(msg)
                else:
                    msg = Float64()
                    msg.data = 0.0
                    self.tilt_pub.publish(msg)
                    rospy.sleep(1.0)
                    msg.data = -0.5
                    self.tilt_pub.publish(msg)


    def move_pillar(self): 
        if self.check_init_pos:
            if len(self.pillar_js.position) > 0:
                if abs(self.pillar_js.position[0]) < 0.01:
                    msg = Float32()
                    msg.data = 0.1
                    self.pillar_pub.publish(msg)
                else:
                    msg = Float32()
                    msg.data = 0.1
                    self.pillar_pub.publish(msg)

    #Sets the ears to solid orange, eyes to >_< 
    def set_led_start(self):
        self.led_eye(command=2, which_part=2, which_feature=3, eye_color=12, mouth_color=12)
        self.led_eye(command=2, which_part=2, which_feature=1, eye_shape=4, mouth_shape=9)
        self.led_ear(command=3, color=3)
    
    #Sets the ears to pulsing white, eyes to blinking
    def set_led_end(self):
        self.led_eye(command=2, which_part=2, which_feature=3, eye_color=11, mouth_color=11)
        self.led_eye(command=2, which_part=2, which_feature=1, eye_shape=0, mouth_shape=5)
        self.led_ear(command=4, color=0)

if __name__ == '__main__':
    rospy.init_node('post_initialization_startup')
    Startup = PostInitializationStartup()
    rospy.sleep(1.0)

    force_initialize = len(sys.argv) > 1 and sys.argv[1] == 'force'
    
    has_been_initialized = False
    if rospy.has_param(init_param):
        has_been_initialized = rospy.get_param(init_param)
        if has_been_initialized and not force_initialize:
            rospy.loginfo("Pan-tilt and pillar systems already initialized. Not running script")
            exit(0)

    Startup.set_led_start()
    rospy.sleep(1.0)
    Startup.move_pan()
    rospy.sleep(1.0)
    Startup.move_tilt()
    #rospy.sleep(2.0)
    #Startup.move_pillar()
    rospy.sleep(1.0)
    Startup.set_led_end()
    rospy.set_param(init_param, True)
    
    exit(0)
