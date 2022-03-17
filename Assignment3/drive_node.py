#!/usr/bin/env python

import rospy # Python library for ROS
from std_msgs.msg import String, UInt16 # String and Unsigned integer message types
from geometry_msgs.msg import Point # Point (x, y, z) message type
import RPi.GPIO as GPIO # Raspberry i GPIO library

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False) # Disable GPIO warnings

# Set variables for the GPIO motor driver pins
motor_left_fw_pin  = 10
motor_left_bw_pin  = 9
motor_right_fw_pin = 8
motor_right_bw_pin = 7

# PWM signal frequency in Hz
pwm_freq = 2000 # Use between 2000 - 20000

# PWM % duty cycle (change these to the values that work best for you)
fw_bw_duty_cycle = 60
turn_duty_cycle  = 40

# Set the GPIO Pin mode to output
GPIO.setup(motor_left_fw_pin, GPIO.OUT)
GPIO.setup(motor_left_bw_pin, GPIO.OUT)
GPIO.setup(motor_right_fw_pin, GPIO.OUT)
GPIO.setup(motor_right_bw_pin, GPIO.OUT)

# Create PWM objects to handle GPIO pins with 'pwm_freq' frequency
motor_left_fw  = GPIO.PWM(motor_left_fw_pin, pwm_freq)
motor_left_bw  = GPIO.PWM(motor_left_bw_pin, pwm_freq)
motor_right_fw = GPIO.PWM(motor_right_fw_pin, pwm_freq)
motor_right_bw = GPIO.PWM(motor_right_bw_pin, pwm_freq)

# Start PWM with a duty cycle of 0 by default
motor_left_fw.start(0)
motor_left_bw.start(0)
motor_right_fw.start(0)
motor_right_bw.start(0)

# Global variables for storing received ROS messages
received_command = ''
last_received_command = ''
received_coord = Point(0, 0, 0)
target_radius = None
MIN_TGT_RADIUS_PERCENT = 0.05
image_width = 0
CENTER_WIDTH_PERCENT = 0.30

def listener():
    # Initialize this node with a the name 'motor_driver'
    rospy.init_node('motor_driver', anonymous=True)

    # Subscribe to the '/command' topic
    rospy.Subscriber('/command', String, commandCallback)

    # Subscribe to the '/target_coord' topic
    rospy.Subscriber('/target_coord', Point, targetCoordCallback)

    # Subscribe to the '/target_radius' topic
    rospy.Subscriber('/target_radius', UInt16, targetRadiusCallback)

    # Subscribe to the '/image_width' topic
    rospy.Subscriber('/image_width', UInt16, imageWidthCallback)

    # Put this node in an inifinite loop to execute when new messages arrive
    rospy.spin()
   
# '/command' topic message handler
def commandCallback(message):
    global received_command
    global last_received_command
    
    received_command = message.data
    
    if received_command == 'forward':
        forward()
    elif received_command == 'backward':
        backward()
    elif received_command == 'left':
        left()
    elif received_command == 'right':
        right()
    elif received_command == 'stop':
        stopMotors()
    elif received_command == 'auto':
        autonomous()
    else:
        print('Unknown command!')
        
    if received_command != last_received_command:
        print('Received command: ' + received_command)
        last_received_command = received_command

# Follow the target in autonomous mode:
def autonomous():
    global image_width
    global target_radius
    global MIN_TGT_RADIUS_PERCENT

    if target_radius >= image_width*MIN_TGT_RADIUS_PERCENT and target_radius <= image_width/3:
        if abs(received_coord.x) <= image_width*CENTER_WIDTH_PERCENT:
            forward()
        elif received_coord.x > 0:
            right()
        elif received_coord.x < 0:
            left()
    else:
        stopMotors()
        print('stopMotors')

# '/image_width' topic message handler
def imageWidthCallback(message):
    global image_width

    image_width = message.data

# '/target_radius' topic message handler
def targetRadiusCallback(message):
    global target_radius

    target_radius = message.data

# '/target_coord' topic message handler
def targetCoordCallback(message):
    # global received_coord

    received_coord.x = message.x
    received_coord.y = message.y
    # print("received_coord = ", received_coord.x, received_coord.y)

# Turn both motors forwards
def forward():
    motor_left_fw.ChangeDutyCycle(fw_bw_duty_cycle)
    motor_left_bw.ChangeDutyCycle(0)
    motor_right_fw.ChangeDutyCycle(fw_bw_duty_cycle)
    motor_right_bw.ChangeDutyCycle(0)

# Turn both motors backwards
def backward():
    motor_left_fw.ChangeDutyCycle(0)
    motor_left_bw.ChangeDutyCycle(fw_bw_duty_cycle)
    motor_right_fw.ChangeDutyCycle(0)
    motor_right_bw.ChangeDutyCycle(fw_bw_duty_cycle)

# Turn left motor backward, right motor forward
def left():
    motor_left_fw.ChangeDutyCycle(0)
    motor_left_bw.ChangeDutyCycle(turn_duty_cycle)
    motor_right_fw.ChangeDutyCycle(turn_duty_cycle)
    motor_right_bw.ChangeDutyCycle(0)

# Turn right motor backward, left motor forward
def right():
    motor_left_fw.ChangeDutyCycle(turn_duty_cycle)
    motor_left_bw.ChangeDutyCycle(0)
    motor_right_fw.ChangeDutyCycle(0)
    motor_right_bw.ChangeDutyCycle(turn_duty_cycle)

# Turn all motors off
def stopMotors():
    motor_left_fw.ChangeDutyCycle(0)
    motor_left_bw.ChangeDutyCycle(0)
    motor_right_fw.ChangeDutyCycle(0)
    motor_right_bw.ChangeDutyCycle(0)

    
if __name__ == '__main__':
    print('Ready to receive commands!')
    listener()
    print('Node is shutting down, stopping motors')
    stopMotors()
