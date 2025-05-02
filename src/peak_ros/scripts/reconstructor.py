#!/usr/bin/env python3

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable

path = "/home/matthew/Desktop/phd_workspaces/kuka_kmr_driver/catkin_ws/src/peak_ros/src/peak_ros/bags/"

file = "pp_full_1745929447090745.pcd"
tof_1 = pd.read_csv(f'{path}{file.split(".")[0]}_tof_data.csv', sep=',')
file = "pp_full_1745929517747275.pcd"
tof_2 = pd.read_csv(f'{path}{file.split(".")[0]}_tof_data.csv', sep=',')
file = "pp_full_1745929597870828.pcd"
tof_3 = pd.read_csv(f'{path}{file.split(".")[0]}_tof_data.csv', sep=',')
file = "pp_full_1745929676195761.pcd"
tof_4 = pd.read_csv(f'{path}{file.split(".")[0]}_tof_data.csv', sep=',')

tof_data = pd.concat([tof_1, tof_2, tof_3, tof_4])

b_scan_pos = tof_data['x'][tof_data['x'] >= 0.4].min()
X = tof_data['x'][tof_data['x'] < 1.1]
Y = tof_data['y'][tof_data['x'] < 1.1]
Z = tof_data['z'][tof_data['x'] < 1.1]

dpi = 300
try:
    fig, ax = plt.subplots(dpi=dpi, figsize=(9, 4))
    im = ax.tricontourf(X, Y, Z, 800, cmap='rainbow', vmin=0.0, vmax=17.0)
    #fig.gca().set_aspect('equal')
    ax.set_title("C-Scan Time-of-Flight")
    ax.set_xlabel("Specimen Position [m]")
    ax.set_ylabel("Array Axis Position [mm]")
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    cbar = plt.colorbar(im, 
                        cax=cax, 
                        label="Depth [mm]",
                        ticks=np.arange(0.0, 17.0, 3.0, dtype=np.float32),
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
    plt.savefig(f'{path}images/tof/composite_reconstruction.png', dpi=dpi)
except Exception as e:
    print(f"Unable to plot ToF C Scan: {e}")
    pass
plt.close()
