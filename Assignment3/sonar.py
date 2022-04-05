#!/usr/bin/env python3
import rospy

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range

pub= rospy.Publisher('/range'.get_caller_id() + 'I heard %s', data.ranges[0])
range_msg = Range()
range_msg.header.stamp = rospy.Time.now()
range_msg.header.frame_id = 'base_laser_link'
range_msg.radiation_type = Range.INFRARED
range_msg.field_of_view = 0.785398
range_msg.min_range = 0.2
range_msg.max_range = 3.0
range_msg.range = data.range[0]
pub.publish(range_msg)

rospy.init_node('sonar_node', anonymous=True)
rospy.Subscriber('/scan', LaserScan, callback)

rospy.spin()

