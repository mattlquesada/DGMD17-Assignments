import rospy
import math, time
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

sub_topic = "/range"
pub_topic = "/cmd_vel"

def callback(data):
    pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
    message_cmd = Twist()
    print("Changing speed and directionality")

    # Change directionality
    print("Changing directionality: index: {}, angular vel: {}".format(data.min_index, get_angular(data.min_index)))
    message_cmd.linear.x = 0
    message_cmd.angular.z = get_angular(data.min_index)
    pub_twist.publish(message_cmd)
    time.sleep(1)

    # Move forward for 2
    print("Changing velocity to 1")
    message_cmd.linear.x = 1
    message_cmd.angular.z = 0
    pub_twist.publish(message_cmd)
    time.sleep(1)

def get_angular(index):
    positive_radians = 22.5
    if index>=0 and index<=22:
        # positive rotation
        return -.3909538*(1-(index/22.5))
    elif index > 22:
        return (index-22.5) * .3909538

# class Follow():
#
#     def __init__(self):
#         self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
#         rospy.loginfo("Publisher set")
#         self._message_cmd = Twist()
#
#     def run(self):
#         self._message_cmd.linear.x = 1
#         self._message_cmd.angular.z = 1
#
#         while True:
#             print("Changing Speed : linear.x: {} angular.z {}".format(self._message_cmd.linear.x, self._message_cmd.angular.z))
#             self.pub_twist.publish()
#             self._message_cmd.linear.x += 1
#             self._message_cmd.angular.z += 1

if __name__ == "__main__":
    rospy.init_node('follow')
    rospy.Subscriber(sub_topic, Range, callback)


