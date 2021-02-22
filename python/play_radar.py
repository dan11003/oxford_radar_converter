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
from radar_io import ProcessFrame,signal_handler
from transform import build_se3_transform
import sys, signal
from decimal import *



parser = argparse.ArgumentParser(description='Play back radar data from a given directory')

parser.add_argument('dir', type=str, help='Directory containing radar data.')

args = parser.parse_args()
signal.signal(signal.SIGINT, signal_handler)


timestamps_path = os.path.join(os.path.join(args.dir,os.pardir, 'radar.timestamps'))
if not os.path.isfile(timestamps_path):
    raise IOError("Could not find timestamps file")
#gt_path = os.path.join(os.path.join(args.dir, os.pardir), '/gt/radar_odometry.csv')
gt_path = os.path.join(args.dir, os.pardir)
print (gt_path)
gt_path = os.path.join(gt_path,'gt/radar_odometry.csv')

print (gt_path)
if not os.path.isfile(gt_path):
    raise IOError("Could not find gt file")

# Cartesian Visualsation Setup
# Resolution of the cartesian form of the radar scan in metres per pixel
cart_resolution = .25
# Cartesian visualisation size (used for both height and width)
cart_pixel_width = 501  # pixels
interpolate_crossover = True

title = "Radar Visualisation Example"
image = None
rospy.init_node('radar_publisher', anonymous=True)
br = CvBridge()

pub = rospy.Publisher('imagetimer', Image,queue_size=10)


radar_timestamps = np.loadtxt(timestamps_path, delimiter=' ', usecols=[0], dtype=np.int64)
GtPoseStamps=[]
with open(gt_path, newline='') as csv_file: #Read radar csv file
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        GtPoseStamps.append(row)


current_frame = 0
pose_init = [0,0,0,0,0,0]
Tpose = build_se3_transform(pose_init)
stamp = rospy.Time.from_sec(0)
try:
    for radar_timestamp in radar_timestamps:
        filename = os.path.join(args.dir, str(radar_timestamp) + '.png')

        if not os.path.isfile(filename):
            raise FileNotFoundError("Could not find radar example: {}".format(filename))

        timestamps, azimuths, valid, fft_data, radar_resolution = load_radar(filename)
        cart_img = radar_polar_to_cartesian(azimuths, fft_data, radar_resolution, cart_resolution, cart_pixel_width, interpolate_crossover)
        print(azimuths)
        image=cart_img


        if current_frame == 0 :
            curr_inc=[0,0,0,0,0,0]
            curr_row = GtPoseStamps[current_frame+1]
            stamp = rospy.Time.from_sec(int(curr_row[1])/100000)
        else :
            curr_row = GtPoseStamps[current_frame]
            stamp = rospy.Time.from_sec(int(curr_row[0])/100000)
            curr_inc = [Decimal(curr_row[2]), Decimal(curr_row[3]), 0, 0, 0, Decimal(curr_row[7])]

        Tpose,tf_transform = ProcessFrame(curr_inc, Tpose, curr_inc, stamp) ## read input transormation and perform fwdkinematics
        msg_image = br.cv2_to_imgmsg(image)
        #msg_image.header.stamp =
        pub.publish()


        # Combine polar and cartesian for visualisation
        # The raw polar data is resized to the height of the cartesian representation
        downsample_rate = 1
        fft_data_vis = fft_data[:, ::downsample_rate]
        resize_factor = float(cart_img.shape[0]) / float(fft_data_vis.shape[0])
        fft_data_vis = cv2.resize(fft_data_vis, (0, 0), None, resize_factor, resize_factor)
        vis = cv2.hconcat((fft_data_vis, fft_data_vis[:, :10] * 0 + 1, cart_img))
     #   vis = cv2.hconcat( cart_img)

    #    cv2.imshow(title, vis * 2.)  # The data is doubled to improve visualisation
        cv2.imshow(title, fft_data_vis * 2.)  # The data is doubled to improve visualisation
        cv2.waitKey(1)

        current_frame+=1
except KeyboardInterrupt:
    print("Press Ctrl-C to terminate while statement")
    quit()
