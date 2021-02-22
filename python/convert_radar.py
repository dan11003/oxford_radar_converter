#!/usr/bin/env python

import argparse
import os
from radar import load_radar, radar_polar_to_cartesian
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
import csv
from radar_io import ProcessFrame,signal_handler,BagWriter
from transform import build_se3_transform
import sys, signal
from cv_bridge import CvBridge, CvBridgeError

from decimal import *



parser = argparse.ArgumentParser(description='Play back radar data from a given directory')

parser.add_argument('dir', type=str, help='Directory containing radar data.')

args = parser.parse_args()
signal.signal(signal.SIGINT, signal_handler)


timestamps_path = os.path.join(os.path.join(args.dir,os.pardir, 'radar.timestamps'))
splitted_dir=args.dir.split("/")
bag_name=splitted_dir[-2]

bag_path = os.path.join(args.dir, bag_name)+".bag"
bw = BagWriter(bag_path)

if not os.path.isfile(timestamps_path):
    raise IOError("Could not find timestamps file")
#gt_path = os.path.join(os.path.join(args.dir, os.pardir), '/gt/radar_odometry.csv')
gt_path = os.path.join(args.dir, os.pardir)
#print (gt_path)
gt_path = os.path.join(gt_path,'gt/radar_odometry.csv')
print ("Loading csv at: "+gt_path)
if not os.path.isfile(gt_path):
    raise IOError("Could not find gt file")

# Cartesian Visualsation Setup
# Resolution of the cartesian form of the radar scan in metres per pixel
cart_resolution = .25
# Cartesian visualisation size (used for both height and width)
cart_pixel_width = 501  # pixels
interpolate_crossover = True

title = "Radar Visualisation Example"

rospy.init_node('radar_publisher', anonymous=True)
br = CvBridge()

pub_cart = rospy.Publisher('/Navtech/Cartesian', Image,queue_size=10)
pub_polar = rospy.Publisher('/Navtech/Polar', Image,queue_size=10)


radar_timestamps = np.loadtxt(timestamps_path, delimiter=' ', usecols=[0], dtype=np.int64)
GtPoseStamps=[]
stamp = rospy.Time.now()
with open(gt_path, newline='') as csv_file: #Read radar csv file
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        GtPoseStamps.append(row)


current_frame = 0
pose_init = [0,0,0,0,0,0]
Tpose = build_se3_transform(pose_init)
curr_row = GtPoseStamps[1]
#stamp = rospy.Time.from_sec(int(curr_row[1])/1000000)
for radar_timestamp in radar_timestamps:


    if current_frame == len(GtPoseStamps) :
        break
    #if current_frame == 240 :
    #    bw.Close()
    #    quit()

    filename = os.path.join(args.dir, str(radar_timestamp) + '.png')
    if not os.path.isfile(filename):
        raise FileNotFoundError("Could not find radar example: {}".format(filename))

    timestamps, azimuths, valid, fft_data, radar_resolution = load_radar(filename)


    #cart_img = radar_polar_to_cartesian(azimuths, fft_data, radar_resolution, cart_resolution, cart_pixel_width,
    #interpolate_crossover)

    if current_frame == 0 :
        curr_inc=[0,0,0,0,0,0]
        curr_row = GtPoseStamps[current_frame+1]
        stamp = rospy.Time.from_sec(int(curr_row[1])/1000000)
    else:
        curr_row = GtPoseStamps[current_frame]
        stamp = rospy.Time.from_sec(int(curr_row[0])/1000000)
        curr_inc = [Decimal(curr_row[2]), Decimal(curr_row[3]), 0, 0, 0, Decimal(curr_row[7])]

    #stamp_ros = rospy.get_rostime()
    stamp_ros = stamp
    #print("scan: ")
    print("stamp: "+str(radar_timestamp))
    print("current_frame: "+str(current_frame))
    #print("stamp: ")
    #print(stamp_ros)
    #print("frame: ")
    #print(current_frame)
    #print("par");
    #print(GtPoseStamps[current_frame])

                # Combine polar and cartesian for visualisation
                # The raw polar data is resized to the height of the cartesian representation
    #downsample_rate = 1
    #fft_data_vis = fft_data[:, ::downsample_rate]
    #resize_factor = float(cart_img.shape[0]) / float(fft_data_vis.shape[0])
    #fft_data_vis = cv2.resize(fft_data_vis, (0, 0), None, resize_factor, resize_factor)
    #print(fft_data_vis)
    #vis = cv2.hconcat((fft_data_vis, fft_data_vis[:, :10] * 0 + 1, cart_img))
    Tpose,tf_transform = ProcessFrame(curr_inc, Tpose, curr_inc, stamp_ros) ## read input transormation and perform fwdkinematics

    fft_data = (255*fft_data)
    img_int = fft_data.astype(np.uint8)

    height = fft_data.shape[0]
    width = fft_data.shape[1]
    #channels = fft_data_vis.shape[2]
    #print('height')
    #print(height)
    #print('width')
    #print(width)
    #print('channels')
    #print(channels)


    #msg_image_cart = br.cv2_to_imgmsg(cart_img)
    msg_image_polar = br.cv2_to_imgmsg(img_int)

    msg_image_polar.header.stamp = stamp_ros
    #msg_image_cart.header.stamp = stamp_ros


    #pub_cart.publish(msg_image_cart)
#    pub_polar.publish(msg_image_polar)
    bw.WriteImage(msg_image_polar, stamp_ros, '/Navtech/Polar')
    bw.WriteTf(tf_transform, stamp_ros)
                #   vis = cv2.hconcat( cart_img)

                #    cv2.imshow(title, vis * 2.)  # The data is doubled to improve visualisation
    #cv2.imshow(title, vis * 2.)  # The data is doubled to improve visualisation
    cv2.imshow(title, img_int)  # The data is doubled to improve visualisation

    #cv2.imshow(title, img_int)  # The data is doubled to improve visualisation
    #cv2.waitKey(1)
    current_frame+=1


                #bw.Close()
