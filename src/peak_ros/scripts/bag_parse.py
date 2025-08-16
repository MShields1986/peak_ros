#!/usr/bin/env python3

# Note that you need to run this on your ROSbag in an environment with the custom messages built
# https://github.com/gavanderhoorn/rosbag_fixer
# python rosbag_fixer/fix_bag_msg_def.py -l inbag.bag outbag.bag

import rosbag   # 1.17
import rospy    # Not needed unless using time windowing
import rospkg   # Not needed unless using the package path finding

import glob, os
#import numpy as np
import pandas as pd
#import matplotlib.pyplot as plt

###############################################################################
# Bag Parsing
###############################################################################

def ascans_bag2dataframe(bag):
    a_scans = [ [msg.count,
                 msg.test_number,
                 msg.dof,
                 msg.channel,
                 msg.amplitudes] for msg in bag ]

    a_scans = pd.DataFrame(a_scans, columns = ['Count',
                                               'Test Number',
                                               'Data Output Format',
                                               'Channel',
                                               'Amplitudes'])
    return a_scans


def observation_bag2dataframe(bag, start_time=None, end_time=None):
    observation = [ [t.to_sec(),
                     msg.header.stamp.to_sec(),
                     msg.header.frame_id,
                     msg.dof,
                     msg.gate_start,
                     msg.gate_end,
                     msg.ascan_length,
                     msg.num_ascans,
                     msg.digitisation_rate,
                     msg.n_elements,
                     msg.element_pitch,
                     msg.inter_element_spacing,
                     msg.vel_wedge,
                     msg.vel_couplant,
                     msg.vel_material,
                     msg.wedge_angle,
                     msg.wedge_depth,
                     msg.couplant_depth,
                     msg.specimen_depth,
                     ascans_bag2dataframe(msg.ascans),
                     # msg.ascans,
                     msg.max_amplitude] for (topic, msg, t) in bag.read_messages(topics=['/peak/a_scans'], start_time=start_time, end_time=end_time) ]

    observation = pd.DataFrame(observation, columns = ['ROS Time Recorded (s)',
                                                       'ROS Time Sent (s)',
                                                       'Frame ID',
                                                       'Data Output Format',
                                                       'Gate Start',
                                                       'Gate End',
                                                       'A Scan Length',
                                                       'Number of A Scans',
                                                       'Digitisation Rate (Mhz)',
                                                       'Number of Focal Laws',
                                                       'Element Pitch (mm)',
                                                       'Inter Element Spacing (mm)',
                                                       'Wedge Velocity (m/s)',
                                                       'Couplant Velocity (m/s)',
                                                       'Material Velocity (m/s)',
                                                       'Wedge Angle (deg)',
                                                       'Wedge Depth (mm)',
                                                       'Couplant Depth (mm)',
                                                       'Specimen Depth (mm)',
                                                       'A Scans',
                                                       'Max Amplitude'])
    return observation


def bscan_bag2dataframe(bag, start_time=None, end_time=None):
    b_scan = [ [t.to_sec(),
                msg.header.stamp.to_sec(),
                msg.header.frame_id,
                msg.height,
                msg.width,
                msg.fields,
                msg.is_bigendian,
                msg.point_step,
                msg.row_step,
                msg.data,
                msg.is_dense] for (topic, msg, t) in bag.read_messages(topics=['/peak/b_scan'], start_time=start_time, end_time=end_time) ]

    b_scan = pd.DataFrame(b_scan, columns = ['ROS Time Recorded (s)',
                                             'ROS Time Sent (s)',
                                             'Frame ID',
                                             'Height',
                                             'Width',
                                             'Fields',
                                             'Is Big Endian',
                                             'Point Step',
                                             'Row Step',
                                             'Data',
                                             'Is Dense'])
    return b_scan

###############################################################################
# Processing
###############################################################################

# Setup
package_path = rospkg.RosPack().get_path('peak_ros')
path = f"{package_path}/bags/"
# path = "/home/matthew/Desktop/phd_workspaces/kuka_kmr_driver/catkin_ws/src/peak_ros/peak_ros/bags/"

# file = "fixed_GATS_400-700_GANS_256_2025-04-10-15-01-14.bag"
# files = [file]

os.chdir(path)
#files = glob.glob("fixed_*")

#for file in files:
file = "fixed_peak_recording_2025-06-10-18-42-55.bag"
print(f'Processing: {file}')

# Time Windowing on 'ROS Time (s)'
time_windowing = False

if time_windowing == True:
    # Arm Motions
    t_min = 1.692628e9 + 395
    t_max = 1.692628e9 + 422

    start_time = rospy.Time(t_min)
    end_time = rospy.Time(t_max)
else:
    start_time = None
    end_time = None

# Parsing
bag_file = path + file
bag = rosbag.Bag(bag_file)

topics = bag.get_type_and_topic_info()[1].keys()

#a_scan_data = observation_bag2dataframe(bag, start_time=start_time, end_time=end_time)
b_scan_data = bscan_bag2dataframe(bag, start_time=start_time, end_time=end_time)

#a_scan_data.to_csv(path_or_buf=file+"_ascan.csv", sep=',', header=True, index=False)
b_scan_data.to_csv(path_or_buf=file+"_bscan.csv", sep=',', header=True, index=False)
