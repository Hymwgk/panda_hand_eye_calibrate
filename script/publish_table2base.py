#!/usr/bin/env python

# marker_calibration.py: Code to publish calibration matrix
# Author: Nishanth Koganti
# Date: 2016/06/15
# Source: https://github.com/osrf/baxter_demos/blob/master/scripts/get_ar_calib.py

import tf
import yaml
import math
import rospy
import numpy as np
from math import pi
import os

def main():
    # initialize ros node
    rospy.init_node('publish_calibration')

    # load calibration file
    #paramFile = rospy.get_param('~parameters_file')
    paramFile = os.environ['HOME']+"/catkin_ws/src/panda_hand_eye_calibrate/config/panda2camera_calibration.yaml"
    with open(paramFile, 'r') as f:
        params = yaml.load(f)

    # parameter initialization
    rot = params['rot']
    child = params['child']
    trans = params['trans']
    parent = params['parent']

    # create tf listener and broadcaster
    tf_broadcaster = tf.TransformBroadcaster()

    # loop rate
    rate = rospy.Rate(100)
    print("Publishing"+parent+" to "+child)
    # Publish transform and marker
    while not rospy.is_shutdown():
        tf_broadcaster.sendTransform(trans, rot, rospy.Time.now(), child, parent)
        rate.sleep()

if __name__=='__main__':
    main()
