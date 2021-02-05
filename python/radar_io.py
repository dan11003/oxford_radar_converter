#!/usr/bin/env python
################################################################################
#
# Copyright (c) 2017 University of Oxford
# Authors:
#  Dan Barnes (dbarnes@robots.ox.ac.uk)
#
# This work is licensed under the Creative Commons
# Attribution-NonCommercial-ShareAlike 4.0 International License.
# To view a copy of this license, visit
# http://creativecommons.org/licenses/by-nc-sa/4.0/ or send a letter to
# Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
#
################################################################################

import argparse
import os
from radar import load_radar, radar_polar_to_cartesian
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
import csv
from transform import build_se3_transform, so3_to_quaternion
from decimal import *
import geometry_msgs
import sys, signal
import rosbag
from std_msgs.msg import Int32, String
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry




class BagWriter:
  def __init__(self, path):
    self.path = path
    print("Writing to bag at path: \n"+path)
    self.bag = rosbag.Bag(self.path, 'w')##

  def WriteImage(self,image, stamp, topic):
    self.bag.write(topic, image, stamp)
  def WriteTf(self,T_tf, stamp):
      tvek = TFMessage()
      tvek.transforms.append(T_tf)
      self.bag.write("/tf", tvek, stamp)
      odom = Odometry()
      odom.header.stamp = T_tf.header.stamp
      odom.header.frame_id = "/world"
      odom.child_frame_id = "/navtech"
      odom.pose.pose.position = T_tf.transform.translation;
      odom.pose.pose.orientation = T_tf.transform.rotation;
      self.bag.write("/gt", odom, stamp)
  def Close(self):
      self.bag.close()

def ProcessFrame(params, Tprev, frame_nr, stamp):    # write Fibonacci series up to n
    #time=gt_row(0) #time is in nanoseconds
    #t.header.stamp = rospy.Time(time)
    #t.header.frame_id = "/world"
    #t.child_frame_id = "/navtech"

    Tinc = build_se3_transform(params)
    Tupd = Tprev*Tinc

    #print("input: ")
    #print(params)
    #print("prev: ")
    #print(Tprev)
    #print("inc: ")
    #print(Tinc)
    #print("Tupd: ")
    #print(Tupd)

    #print("ros time: ")
    #print(stamp)



    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "/world"
    t.header.stamp = stamp
    t.child_frame_id = "/navtech"
    t.transform.translation.x = Tupd[0,3]
    t.transform.translation.y = Tupd[1,3]
    t.transform.translation.z = 0.0
    #print(t.transform.translation)

    qupd = so3_to_quaternion(Tupd[0:3,0:3])
    t.transform.rotation.x = qupd[1]
    t.transform.rotation.y = qupd[2]
    t.transform.rotation.z = qupd[3]
    t.transform.rotation.w = qupd[0]

    return Tupd,t


def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    sys.exit(0)
