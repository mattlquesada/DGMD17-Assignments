#!/usr/bin/env python3
import rospy
import tf
from tf.transformations import euler_from_quaternion
rospy.init_nod("tf_example")
listener = tf.TransformListener()
while not rospy.is_shutdown():
   try:
         (trans,rot) = listener.lookupTransform(
               "/odom","/base_footprint".rospy.Time(0)
         )
   except (tf.lookupException, tf.ConnectivityException,
tf.ExtrapolationException):
      continue
   robot_x = trans[0]
   robot_y = trans[1]
   (x, y, z, w) = rot
   #from euler_from_quaternion, we get: roll,pitch and yaw, but we only need yaw
   (_,_,yaw) = euler_from_quaternion([(x, y, z, w)])
   print(f"X: {robot_x:.3f}\tY: {robot_y:.3f}\tYaw: {yaw:.3f}")
   rospy.sleep(0.1)