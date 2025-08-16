#!/usr/bin/env python3

import ast
import struct
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
# from mpl_toolkits.axes_grid1 import make_axes_locatable

# path = "/home/matthew/Desktop/phd_workspaces/kuka_kmr_driver/catkin_ws/src/peak_ros/src/peak_ros/bags/"
path = "/media/matthew/b2603e49-ec47-413e-94db-d319afa80e0a/home/matthew/Desktop/phd_data/ut/"
file = "fixed_peak_recording_2025-06-10-18-42-55_bscan.csv"

chunk = pd.read_csv(f'{path}{file}', sep=',', chunksize=1000)
data = pd.concat(chunk)

# #######################################################################################
# Going to need to get the time stamps or observation no. of pcl_crop_box_2 to ltpa 0.4 m
# #######################################################################################
# Max len(data)
obs = [120,
       970,
       1800,
       2652,
       3496,
       4400, # approx from here on
       5300,
       6200,
       7100,
       8000]

x_biglist = []
y_biglist = []
z_biglist = []
amp_biglist = []

pass_counter = 0
for obs_i in obs:
    # Step 1: Safely evaluate the literal into bytes
    lit_data = ast.literal_eval(data['Data'][obs_i])
    
    # Step 2: Parse into floats (example with 32-bit little-endian)
    points = struct.unpack(f"<{len(lit_data)//4}f", lit_data[:(len(lit_data)//4)*4])
    
    x_list = []
    y_list = []
    z_list = []
    amp_list = []
    
    counter = 1
    for point in points:
        if counter == 1:
            x_list.append(point)
            x_biglist.append(point)
            counter+=1
        elif counter == 2:
            y_list.append(point)
            y_biglist.append(point + pass_counter * 0.05016)
            counter+=1
        elif counter == 3:
            z_list.append(point)
            z_biglist.append(point / 2940 * 1000 * 1000) # m to us
            counter+=1
        elif counter == 4:
            amp_list.append(point)
            amp_biglist.append(point)
            counter = 1
            
    pass_counter += 1

lines = [1 * 0.05016,
         2 * 0.05016,
         3 * 0.05016,
         4 * 0.05016,
         5 * 0.05016,
         6 * 0.05016,
         7 * 0.05016,
         8 * 0.05016,
         9 * 0.05016]

dpi = 300

X = y_biglist
Y = z_biglist
Z = amp_biglist

try:
    plt.figure(figsize=(9, 5), dpi=dpi)
    plt.gca().invert_yaxis()
    plt.tricontourf(X, Y, Z, 800, cmap='viridis', vmin=-1.0, vmax=1.0)
    plt.colorbar(label="Normalised Amplitude",
                 ticks=np.arange(-1.0, 1.0, 0.2, dtype=np.float32),
                 boundaries=[-1.0, 1.05]
                 )
    plt.vlines(lines, ymin=min(Y), ymax=max(Y), colors='k', linestyles='--')
    plt.title("10 B-Scans at Specimen Position 0.4 m")
#    plt.title(f"B Scan at Time {str(round(data['ROS Time Sent (s)'][obs_i], 3))} s")
    plt.xlabel("Array Axis [m]")
    plt.ylabel("Time [us]")
    plt.tight_layout()
    #plt.show()
    path = "/home/matthew/Desktop/PhD/bindt_2025_presentation/"
    plt.savefig(f'{path}images/bscan_composite_01.png', dpi=dpi)

except Exception as e:
    print(f"Unable to plot exploratory b scan: {e}")
    pass
plt.close()
