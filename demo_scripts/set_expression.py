#!/usr/bin/env python


import math
import rospy
import actionlib
from poli_msgs.srv import LedEye, LedEyeRequest
from std_msgs.msg import Float64

"""This function sets the eye/mouth shape and color through /led_eye"""
def set_expression(eye_shape, eye_color, eye_mapping, mouth_shape, mouth_color, mouth_mapping, color_mapping):
    change_expression = rospy.ServiceProxy("/led_eye",LedEye)
    if mouth_shape != "":
       change_expression(command=LedEyeRequest.UPDATE, which_part=LedEyeRequest.MOUTH, which_feature=LedEyeRequest.SHAPE, mouth_shape=mouth_mapping[mouth_shape])
    if mouth_color != "":
       change_expression(command=LedEyeRequest.UPDATE, which_part=LedEyeRequest.MOUTH, which_feature=LedEyeRequest.COLOR, mouth_color=color_mapping[mouth_color])
    if eye_shape != "":
       change_expression(command=LedEyeRequest.UPDATE, which_part=LedEyeRequest.EYES, which_feature=LedEyeRequest.SHAPE, eye_shape=eye_mapping[eye_shape])
    if eye_color != "":
        change_expression(command=LedEyeRequest.UPDATE, which_part=LedEyeRequest.EYES, which_feature=LedEyeRequest.COLOR, eye_color=color_mapping[eye_color])
       

def set_pan(pan_angle):
    if pan_angle != "":
        pan_angle = int(pan_angle)

        # validate
        if pan_angle < -90 or pan_angle > 90:
            print("angle outside range. Clipping to valid range")
            pan_angle = min(max(pan_angle, -90), 90)
            print("clipped pan angle: {}".format(pan_angle))
        # degrees to radians
        pan_angle = math.radians(pan_angle)
        pan_angle = -pan_angle
        
        pan_publisher.publish(pan_angle)
        
def set_tilt(tilt_angle):
    if tilt_angle != "":
        tilt_angle = float(tilt_angle)

        # validate
        if tilt_angle < 0 or tilt_angle > 1:
            print("angle outside range. Clipping to valid range")
            tilt_angle = min(max(tilt_angle, 0), 1)
            print("clipped tilt angle: {}".format(tilt_angle))
        # degrees to radians
        tilt_angle = -tilt_angle
        
        tilt_publisher.publish(tilt_angle)

if __name__ == "__main__":

    rospy.init_node("set_expression")
    rospy.wait_for_service("/led_eye")
    pan_publisher = rospy.Publisher("/pan_motor/position_controller/command", Float64, queue_size=1)
    tilt_publisher = rospy.Publisher("/tilt_motor/position_controller/command", Float64, queue_size=1)
    rospy.loginfo("Expression setter ready to go")
    
    eye_mapping = {"normal": LedEyeRequest.NORMAL, "close": LedEyeRequest.CLOSE, "cry": LedEyeRequest.CRY, "squint": LedEyeRequest.SQUINT, "wink": LedEyeRequest.WINK, "mad": LedEyeRequest.MAD, "sad": LedEyeRequest.SAD, "side eye": LedEyeRequest.SIDERIGHT, "heart": LedEyeRequest.HEART, "sleepy": LedEyeRequest.SLEEPY, "dead": LedEyeRequest.DEAD}
    mouth_mapping = {"flat": LedEyeRequest.FLAT, "grin": LedEyeRequest.GRIN, "grimace": LedEyeRequest.GRIMACE, "open": LedEyeRequest.OPEN, "smile": LedEyeRequest.SMILE, "frown": LedEyeRequest.FROWN, "big open": LedEyeRequest.BIGOPEN, "squiggle": LedEyeRequest.SQUIGGLE, "whistle": LedEyeRequest.WHISTLE}
    color_mapping = {"white": 0, "dark green": 1, "teal": 2, "light blue": 3, "blue": 4, "dark blue": 5, "purple": 6, "pink": 7, "hot pink": 8, "red": 9, "orange": 10, "yellow": 11, "light green": 12}
    
    print "Eye:", eye_mapping.keys()
    print "--------------------------------------------------------------------------"
    print "Mouth:", mouth_mapping.keys()
    print "------------------------------------------------------------------------------"
    print "Colors:", color_mapping.keys()
    print "------------------------------------------------------------------------------"
    
    
    eye_shape = raw_input("New eye shape (ENTER if no change):")
    while eye_shape not in eye_mapping.keys():
        eye_shape = raw_input("Unknown key, please choose another:")
    eye_color = raw_input("New eye color (ENTER if no change):")
    while eye_color not in color_mapping.keys():
        eye_color = raw_input("Unknown key, please choose another:")
    mouth_shape = raw_input("New mouth shape (ENTER if no change):")
    while mouth_shape not in mouth_mapping.keys():
        mouth_shape = raw_input("Unknown key, please choose another:")
    mouth_color = raw_input("New mouth color (ENTER if no change):")
    while mouth_color not in color_mapping.keys():
        mouth_color = raw_input("Unknown key, please choose another:")
    pan_angle = raw_input("New pan angle in degrees, -90 to 90, positive is turning to the right (ENTER if no change):")
    tilt_angle = raw_input("New tilt angle, 0 to 1, 1 is up, 0 is down (ENTER if no change):")
    
    set_expression(eye_shape, eye_color, eye_mapping, mouth_shape, mouth_color, mouth_mapping, color_mapping)
    set_pan(pan_angle)
    set_tilt(tilt_angle)
