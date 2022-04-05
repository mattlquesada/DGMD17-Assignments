import rospy
import math, time
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class Follow():

    def __init__(self):

        # - subscribe to the source_vel coming from teleop_twist
        self.sub_center = rospy.Subscriber("/source_vel", Twist, self.update_source)
        rospy.loginfo("Source Command Subscribers set")

        # - we will publish the total command to the vehicle here
        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        rospy.loginfo("Publisher set")
        self._message_cmd = Twist()

    def run(self):
        self._message_cmd.linear.x = 1
        self._message_cmd.angular.z = 1

        while True:
            print("Changing Speed : linear.x: {} angular.z {}".format(self._message_cmd.linear.x, self._message_cmd.angular.z))
            self.pub_twist.publish()
            time.sleep(3)
            self._message_cmd.linear.x += 1
            self._message_cmd.angular.z += 1

if __name__ == "__main__":
    rospy.init_node('follow')
    follow = Follow()
    follow.run()

