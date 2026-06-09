#!/usr/bin/env python3

# Parse a ROS 2 bag (sqlite3 or mcap) recorded by peak_ros into CSV.
# Run this in an environment where the peak_ros interfaces are built and sourced
# (e.g. `source install/setup.bash`) so the custom messages can be deserialised.

import os

import pandas as pd

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from ament_index_python.packages import get_package_share_directory

###############################################################################
# Bag Parsing
###############################################################################

def _open_reader(bag_path):
    storage_id = "mcap" if bag_path.endswith(".mcap") else "sqlite3"
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                    output_serialization_format="cdr"),
    )
    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    return reader, type_map


def _read_messages(bag_path, topic):
    reader, type_map = _open_reader(bag_path)
    msg_type = get_message(type_map[topic])
    while reader.has_next():
        (read_topic, data, t_ns) = reader.read_next()
        if read_topic == topic:
            yield deserialize_message(data, msg_type), t_ns
    del reader


def _stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def ascans_msgs2dataframe(ascans):
    a_scans = [[a.count,
                a.test_number,
                a.dof,
                a.channel,
                a.amplitudes] for a in ascans]

    a_scans = pd.DataFrame(a_scans, columns=['Count',
                                             'Test Number',
                                             'Data Output Format',
                                             'Channel',
                                             'Amplitudes'])
    return a_scans


def observation_bag2dataframe(bag_path, topic='/peak/a_scans'):
    observation = [[t_ns * 1e-9,
                    _stamp_to_sec(msg.header.stamp),
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
                    ascans_msgs2dataframe(msg.ascans),
                    msg.max_amplitude] for (msg, t_ns) in _read_messages(bag_path, topic)]

    observation = pd.DataFrame(observation, columns=['ROS Time Recorded (s)',
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


def bscan_bag2dataframe(bag_path, topic='/peak/b_scan'):
    b_scan = [[t_ns * 1e-9,
               _stamp_to_sec(msg.header.stamp),
               msg.header.frame_id,
               msg.height,
               msg.width,
               msg.fields,
               msg.is_bigendian,
               msg.point_step,
               msg.row_step,
               bytes(msg.data),
               msg.is_dense] for (msg, t_ns) in _read_messages(bag_path, topic)]

    b_scan = pd.DataFrame(b_scan, columns=['ROS Time Recorded (s)',
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
package_path = get_package_share_directory('peak_ros')
path = f"{package_path}/bags/"

os.chdir(path)

# ROS 2 bags are directories (sqlite3) or .mcap files. Point at the bag accordingly.
bag = "peak_recording_2025-06-10-18-42-55"
bag_path = path + bag
print(f'Processing: {bag}')

# Parsing
# a_scan_data = observation_bag2dataframe(bag_path, topic='/peak/a_scans')
b_scan_data = bscan_bag2dataframe(bag_path, topic='/peak/b_scan')

# a_scan_data.to_csv(path_or_buf=bag + "_ascan.csv", sep=',', header=True, index=False)
b_scan_data.to_csv(path_or_buf=bag + "_bscan.csv", sep=',', header=True, index=False)
