#!/usr/bin/env python
import rospy

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range

pub_topic = "/range"
sub_topic = "/scan"

pub = rospy.Publisher(pub_topic, Range, queue_size=10)


def callback(data):

    range_msg = Range()
    range_msg.min_index = data.ranges.index(min(data.ranges))
    rospy.loginfo(rospy.get_caller_id() + "I heard indx: {}, distance: {}".format(min_index, data.ranges[min_index]))
    if data.ranges[range_msg.min_index] < 30:
        rospy.loginfo("object detected")
    else:
        rospy.loginfo("no object detected")

    range_msg.header.stamp = rospy.Time.now()
    range_msg.header.frame_id = data.header.frame_id
    range_msg.radiation_type = Range.ULTRASOUND
    range_msg.field_of_view = 0.7819076
    range_msg.min_range = 0.1
    range_msg.max_range = 30.0
    range_msg.range = data.ranges.index(min(data.ranges))
    pub.publish(range_msg)


rospy.init_node("sonar_node", anonymous=True)
rospy.Subscriber(sub_topic, LaserScan, callback)

rospy.spin()
