#!/usr/bin/env python

"""
This is a high level driver for the EPOS tilt controller for the PoliV2 platform.
This file also launches a script that moves the pan/tilt/pillar/led systems for visual inspection.

The specific purpose of this driver is to ensure that the tilt motor's position encoders 
have been initialized properly. The encoders are set on power-on to the motor.

This driver checks if the tilt LIMIT SWITCH is pressed.
If so, it launches the tilt driver and checks the reported position of its encoders.
If its position value reports 0.0 +/- TOLERANCE the driver is kept alive.
If its out of TOLERANCE the driver is killed, error messages are logged, and this driver exits.

There is a persistance parameter INIT_PARAM that is used to track if this driver has been run since 
the robot has been started. This assumes that the tilt motor's power is tied exclusively with the 
rest of the robot.


Author Maxwell Svetlik

"""

import rospy
import roslaunch
import rospkg
from std_msgs.msg import Int16
from sensor_msgs.msg import JointState

TOLERANCE = 0.01 #0.007
init_param = "/tilt_motor/encoders_initialized"

class LaunchTiltMotor(object):

    def cb_limit_switch(self, data):
        self.switch_pressed = bool(data.data)
        
    def cb_joint_states(self, data):
        if len(data.position) > 0:
            self.joint_states = data

    def __init__(self):
        self.switch_pressed = False
        self.joint_states = JointState()
        
        if rospy.has_param(init_param):
            has_been_initialized = rospy.get_param(init_param)
            rospy.loginfo("Tilt motor has been previously initialized.")
        else:
            has_been_initialized = False

        rospy.init_node('launch_tilt_motor', anonymous=True)
        rospy.Subscriber("/pillar/limit_switch", Int16, self.cb_limit_switch)
        rospy.Subscriber("/tilt_motor/joint_states", JointState, self.cb_joint_states)

        rospack = rospkg.RosPack()
        
        # Setup roslaunch for tilt motor
        launch_file_name_tilt_control = "/launch/tilt_control.launch"

        tilt_control_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(tilt_control_uuid)
        launch = roslaunch.parent.ROSLaunchParent(tilt_control_uuid, [rospack.get_path('poli_pan_tilt')+launch_file_name_tilt_control])
        launch.start()
        
        # wait for EPOS driver to start
        tilt_encoder = 0.0
        if not has_been_initialized:
            while not has_been_initialized and not rospy.is_shutdown():
                rospy.sleep(1.0)
                if len(self.joint_states.position) > 0:
                    tilt_encoder = self.joint_states.position[0]
                    has_been_initialized = True
                else:
                    rospy.logwarn_throttle(5, "Waiting for tilt motor joint state to be initialized...")
        
        if rospy.is_shutdown():
            return

        rospy.sleep(2.0)

        rospy.loginfo("Tilt encoder: {}; Limit switch state: {}".format(tilt_encoder, self.switch_pressed))
        
        rospy.loginfo("initialized: {}; tilt_encoder: {}; switch_pressed: {};".format(has_been_initialized, tilt_encoder, self.switch_pressed))
        if has_been_initialized and (tilt_encoder <= TOLERANCE and tilt_encoder >= -TOLERANCE and self.switch_pressed):
            #life is good
            rospy.set_param(init_param, "true")
            rospy.loginfo("Tilt motor in good state. Continuing to run.")

            # Setup roslaunch for post-initialization movement script
            launch_file_name_initscript = "/launch/torso/post_initialization_movement.launch"
            post_initialization_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(post_initialization_uuid)
            post_initialization_launch = roslaunch.parent.ROSLaunchParent(post_initialization_uuid, [rospack.get_path('poli2_launch')+launch_file_name_initscript])
            post_initialization_launch.start()
            
            while not rospy.is_shutdown():
                rospy.spin()
        else:
            #tilt motor not within spec
            rospy.logerr("Tilt motor does not have correctly initialized encoders. Encoder at {}".format(tilt_encoder))
            rospy.logerr("Please tilt the head down, disconnect tilt motor power and reconnect.")
            launch.shutdown()
            if rospy.has_param(init_param):
                rospy.delete_param(init_param)

if __name__ == '__main__':
    ltm = LaunchTiltMotor()
