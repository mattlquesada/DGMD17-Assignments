#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

received_command = ''
last_received_command = ''

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    rospy.init_node('motor_driver', anonymous=True)

    rospy.Subscriber('/command', String, commandCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
# Message handler
def commandCallback(commandMessage):
    global received_command
    global last_received_command
    
    received_command = commandMessage.data
    
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
    else:
        print('Unknown command!')
        
    if received_command != last_received_command:
        print('Received command: ' + received_command)
        last_received_command = received_command

# -- Differential drive:
# Turn all motors off
def stopMotors():
    # Here goes the code for driving the motors
    pass

# Turn both motors forwards
def forward():
    # Here goes the code for driving the motors
    pass

# Turn both motors backwards
def backward():
    # Here goes the code for driving the motors
    pass

# Turn left motor backward, right motor forward
def left():
    # Here goes the code for driving the motors
    pass

# Turn right motor backward, left motor forward
def right():
    # Here goes the code for driving the motors
    pass

if __name__ == '__main__':
    print('Ready to receive commands!')
    listener()
    print('Node is shutting down, stopping motors')
    stopMotors()
