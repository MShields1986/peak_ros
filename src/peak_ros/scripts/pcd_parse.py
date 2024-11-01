#!/usr/bin/env python3

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable

path = "/home/matthew/Desktop/phd_workspaces/kuka_kmr_driver/catkin_ws/src/peak_ros/src/peak_ros/bags/"

#file = "pp_full_1745422377521490.pcd" # Too Short   - Near and Far
#file = "pp_full_1745422390265942.pcd" # Done        - Near and Far
#file = "pp_full_1745422641176811.pcd" # Weak signal - Far Stringer Far
#file = "pp_full_1745423618327693.pcd" # Weak signal - Far Stringer Far
#file = "pp_full_1745425047850435.pcd" # Done        - Far Stringer Far
#file = "pp_full_1745425284821404.pcd" # Done        - Near Far Stringer

#file = "pp_full_1745929447090745.pcd"
file = "pp_full_1745929517747275.pcd"
#file = "pp_full_1745929597870828.pcd"
#file = "pp_full_1745929676195761.pcd"

with open(path + file, "r") as pcd_file:
    lines = [line.strip().split(" ") for line in pcd_file.readlines()]

is_data = False
data = []
for line in lines:
    if line[0] == 'FIELDS':
        cols = line
        cols.pop(0)
    elif line[0] == 'DATA':
        is_data = True
        continue
    elif is_data:
        data.append(line)

del(line, lines, is_data, pcd_file)
data = pd.DataFrame(data, columns = cols)
data = data.astype(np.float32)

# depth = 0.0107457 # median
depths = data["z"].unique() 

fw_depth = 1.6 # mm
#fw_depth = 0.0 # mm
dpi = 300

data["Amplitudes"] = (data["Amplitudes"] - 0.5) * 2
data["y"] = data["y"] * 1000 # to mm
data["z"] = data["z"] * 1000 # to mm

# offset to front wall
for_plotting = data
for_plotting["z"] = for_plotting["z"] - fw_depth

#for_plotting = data

for_plotting = for_plotting[for_plotting["z"] < (17.0 - fw_depth)]
for_plotting = for_plotting[for_plotting["z"] > 6.0]
for_plotting = for_plotting.reset_index(drop=True)

###############################################################################
# Exploratory B Scan
###############################################################################

# Get closest value to 0.4
b_scan_pos = data['x'][data['x'] >= 0.4].min()

exp_bscan = for_plotting[for_plotting['x'] == b_scan_pos]
exp_bscan['z'] = exp_bscan['z']/2940*1000

try:
    plt.figure(figsize=(9, 5), dpi=dpi)
    plt.gca().invert_yaxis()
    plt.tricontourf(exp_bscan['y'], exp_bscan['z'], exp_bscan['Amplitudes'], 800, cmap='viridis', vmin=-1.0, vmax=1.0)
    plt.colorbar(label="Normalised Amplitude",
                 ticks=np.arange(-1.0, 1.0, 0.2, dtype=np.float32),
                 boundaries=[-1.0, 1.05]
                 )
    plt.title(f"B Scan at Specimen Position {str(round(b_scan_pos, 1))} m")
    plt.xlabel("Array Positon (mm)")
    plt.ylabel("Time (us)")
    plt.show()
    #plt.savefig(f'{path}images/tof/exp_b_scan_{file.split(".")[0]}.png', dpi=dpi)
except Exception as e:
    print(f"Unable to plot exploratory b scan: {e}")
    pass
plt.close()

###############################################################################
# C Scan
###############################################################################

for_plotting['Amplitudes'] = for_plotting['Amplitudes'].abs()

unique_locs = for_plotting.groupby(['x','y']).size().reset_index().rename(columns={0:'count'})

tof_data = []
i = 0
n = len(for_plotting['x'].unique()) * len(for_plotting['y'].unique())
for row in unique_locs.iterrows():
    x = row[1]['x']
    y = row[1]['y']

    # TODO: This takes a long time as copies the df
    #       using .where() is around twice as bad
    tmp_data = for_plotting['Amplitudes'][
                    (for_plotting['x'] == x) & 
                    (for_plotting['y'] == y)
                    ]

    tmp_data = for_plotting['Amplitudes'].where(
                    (for_plotting['x'] == x) & 
                    (for_plotting['y'] == y)
                    )
    
    #tmp_min = tmp_data.min()
    tmp_max = tmp_data.max()
    
    # TODO: This takes a long time as copies the df
    #       using .where() is around twice as bad
    tof = for_plotting['z'][
        (for_plotting['x'] == x) & 
        (for_plotting['y'] == y) & 
        (for_plotting['Amplitudes'] == tmp_max)
        ]

    row = {'x': x, 'y': y, 'z': tof.iloc[0], 'Amplitudes': tmp_max}

    tof_data.append(row)

    i += 1
    if i % 100 == 0:
        print(f"Processed {i} / {n} ToF Samples")


tof_data = pd.DataFrame(tof_data, columns = cols)

tof_data_orig = tof_data

tof_data.to_csv(f'{path}{file.split(".")[0]}_tof_data.csv', index=False, sep=',', na_rep='')

#tof_data = tof_data[tof_data['z'] != tof_data['z'].max()]
#tof_data[tof_data['z'] <= 12.3] = 0.1
#tof_data = tof_data_orig


X = tof_data['x']
Y = tof_data['y']
Z = tof_data['z']

try:
    fig, ax = plt.subplots(dpi=dpi, figsize=(9, 4))
    im = ax.tricontourf(X, Y, Z, 800, cmap='rainbow', vmin=0.0, vmax=17.0)
    ax.set_title("C-Scan Time-of-Flight")
    ax.set_xlabel("Specimen Position [m]")
    ax.set_ylabel("Array Position [mm]")
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    cbar = plt.colorbar(im, 
                        cax=cax, 
                        label="Depth [mm]",
                        ticks=np.arange(0.0, 17.0, 1.0, dtype=np.float32),
                        boundaries=[0.0, 17.0]
                        )
    ax.axvline(b_scan_pos,
               color='grey',
               linestyle='--',
               linewidth=2,
               label=f"B-Scan Position ({str(round(b_scan_pos, 1))} m)"
               )
    ax.legend(loc='upper right')
    plt.tight_layout()
    #plt.show()
    plt.savefig(f'{path}images/tof/{file.split(".")[0]}.png', dpi=dpi)
except Exception as e:
    print(f"Unable to plot ToF C Scan: {e}")
    pass
plt.close()

###############################################################################
# For Real B Scan
###############################################################################

bscan = data[data['x'] == b_scan_pos]
bscan = bscan[bscan["z"] < (17.0 - fw_depth)]
bscan['z'] = bscan['z'] + fw_depth
bscan['z'] = bscan['z']/2940*1000
bscan = bscan.reset_index(drop=True)

try:
    plt.figure(figsize=(7, 5), dpi=dpi)
    #plt.gca().set_aspect('equal')
    plt.gca().invert_yaxis()
    plt.tricontourf(bscan['y'],
                    bscan['z'],
                    bscan['Amplitudes'],
                    800,
                    cmap='viridis',
                    vmin=-1.0,
                    vmax=1.0
                    )
    plt.colorbar(label="Normalised Amplitude",
                 ticks=np.arange(-1.0, 1.0, 0.2, dtype=np.float32),
                 boundaries=[-1.0, 1.05]
                 )
    plt.title(f"B Scan at Specimen Position {str(round(b_scan_pos, 1))} m")
    plt.xlabel("Array Positon [mm]")
    plt.ylabel("Time [us]")
    #plt.show()
    plt.savefig(f'{path}images/tof/b_scan_{file.split(".")[0]}.png', dpi=dpi)
except Exception as e:
    print(f"Unable to plot B Scan: {e}")
    pass
plt.close()

###############################################################################
# Amplitude C Scans
###############################################################################
"""
for depth in depths:
    for_plotting = data[data["z"] == depth]
    #for_plotting = for_plotting[for_plotting["x"] < 1.15]

    X = for_plotting['x']
    Y = for_plotting['y'] * 1000
    Z = for_plotting['Amplitudes']

    try:
        plt.figure(figsize=(9, 4), dpi=dpi)
        plt.tricontourf(X, Y, Z, 800, cmap='viridis', vmin=0.0, vmax=1.0)
        plt.colorbar(label="Normalised Amplitude",
                     ticks=np.arange(0.0, 1.0, 0.1, dtype=np.float32),
                     boundaries=[0.0, 1.0]
                     )
        plt.title(f"C Scan at {round((depth * 1000) - fw_depth, 3)} mm Deep")
        plt.xlabel("Specimen Positon (m)")
        plt.ylabel("Array Position (mm)")
        # plt.show()
        plt.savefig(f'{path}images/{file.split(".")[0]} - {round((depth * 1000) - fw_depth, 3)} mm Deep.png', dpi=dpi)
    except Exception as e:
        print(f"Unable to plot depth {round(depth * 1000, 3)} mm: {e}")
        pass
    plt.close()
"""