#!/usr/bin/env python
# -*- coding: utf8 -*-#

import rospy
from std_msgs.msg import Bool


state = Bool()
state.data = True
pub = rospy.Publisher('topic', Bool, queue_size=10)
rospy.init_node('multi_box_filter_state', anonymous=True)
r = rospy.Rate(10)
while not rospy.is_shutdown():
    pub.publish(state)
    r.sleep()
