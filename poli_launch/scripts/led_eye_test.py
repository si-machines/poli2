#!/usr/bin/env python

import rospy
from poli_msgs.srv import LedEye

if __name__ == "__main__":
	print "Requesting led_eye"
	rospy.wait_for_service('led_eye')
	try:
		led_eye = rospy.ServiceProxy('led_eye', LedEye)
		print "Trying led_eye"
		led_eye(command=2,which_part=1,which_feature=1,mouth_shape=1)
		print "Completed led_eye"
	except rospy.ServiceException, e:
		print "Failed: printing error"
		print "Service call failed: %s"%e
