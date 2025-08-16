#!/usr/bin/env python3

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable

# path = "/home/matthew/Downloads/ec_old/"

# path = "/home/matthew/Desktop/PhD/videos/250812/"

path = "/media/matthew/b2603e49-ec47-413e-94db-d319afa80e0a/home/matthew/Desktop/phd_data/ut/"

# file = "1727973611228394_axial.pcd"

# file = "1755002810444365_axial.pcd"
# file = "1755002810444365_axial.pcd"

#file = "1747757835589812.pcd"
#file = "1749212683045125.pcd"
#file = "1749481910941825.pcd"
file = "1749578690843707.pcd"

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

data['x'] = data['x'] + data['x'].min() * -1
data['y'] = data['y'] + data['y'].min() * -1
data['z'] = data['z'] + data['z'].min() * -1

dpi = 300

###############################################################################
# Imaginary
###############################################################################

X = data['x']
Y = data['y']

try:
    Z = data['Imaginary']

    fig, ax = plt.subplots(dpi=dpi, figsize=(9, 4))
#    im = ax.tricontourf(X, Y, Z, 800, cmap='rainbow')
    im = ax.tricontourf(X, Y, Z, 800, cmap='rainbow', vmin=Z.min(), vmax=150.0)
    ax.set_title("C-Scan Reconstruction - Imaginary Component")
    ax.set_xlabel("Scan Axis [m]")
    ax.set_ylabel("Array Axis [m]")
    ax.set_aspect('equal')
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    cbar = plt.colorbar(im, 
                        cax=cax, 
                        label="Impedance",
                        ticks=np.arange(-1000.0, 1000.0, 50.0, dtype=np.float32),
                        boundaries=[Z.min(), Z.max()]
                        )
#    ax.axvline(b_scan_pos,
#               color='grey',   
#               linestyle='--',
#               linewidth=2,
#               label=f"B-Scan Position ({str(round(b_scan_pos, 1))} m)"
#               )
#    ax.legend(loc='upper right')
    plt.tight_layout()
    plt.show()
    #plt.savefig(f'{path}images/tof/{file.split(".")[0]}.png', dpi=dpi)
except Exception as e:
    print(f"Unable to plot C Scan: {e}")
    pass
#plt.close()

###############################################################################
# Real
###############################################################################

try:
    Z = data['Real']

    fig, ax = plt.subplots(dpi=dpi, figsize=(9, 4))
#    im = ax.tricontourf(X, Y, Z, 800, cmap='rainbow')
    im = ax.tricontourf(X, Y, Z, 800, cmap='rainbow', vmin=Z.min(), vmax=150.0)
    ax.set_title("C-Scan Reconstruction - Real Component")
    ax.set_xlabel("Scan Axis [m]")
    ax.set_ylabel("Array Axis [m]")
    ax.set_aspect('equal')
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    cbar = plt.colorbar(im, 
                        cax=cax, 
                        label="Impedance",
                        ticks=np.arange(-1000.0, 1000.0, 50.0, dtype=np.float32),
                        boundaries=[Z.min(), Z.max()]
                        )
#    ax.axvline(b_scan_pos,
#               color='grey',   
#               linestyle='--',
#               linewidth=2,
#               label=f"B-Scan Position ({str(round(b_scan_pos, 1))} m)"
#               )
#    ax.legend(loc='upper right')
    plt.tight_layout()
    plt.show()
    #plt.savefig(f'{path}images/tof/{file.split(".")[0]}.png', dpi=dpi)
except Exception as e:
    print(f"Unable to plot C Scan: {e}")
    pass
#plt.close()

###############################################################################
# ToF
###############################################################################

try:
    b_scan_pos = 0.4
    X = data.query('x < 1.6')['x']
    Y = data.query('x < 1.6')['y']
    Z = data.query('x < 1.6')['TimeofFlight'] * 1000 - 3.5 # Depth to front wall

    fig, ax = plt.subplots(dpi=dpi, figsize=(9, 3))
    im = ax.tricontourf(X, Y, Z, 800, cmap='rainbow')
    # im = ax.tricontourf(X, Y, Z, 800, cmap='rainbow', vmin=0.0, vmax=20.0)
    ax.set_title("C-Scan Reconstruction - Time of Flight")
    ax.set_xlabel("Scan Axis [m]")
    ax.set_ylabel("Array Axis [m]")
    ax.set_aspect('equal')
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    cbar = plt.colorbar(im, 
                        cax=cax, 
                        label="Depth of Detection [mm]",
                        ticks=np.arange(-100.0, 100.0, 2.5, dtype=np.float32),
                        boundaries=[0.0, 20.0]
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
    path = "/home/matthew/Desktop/PhD/bindt_2025_presentation/"
    plt.savefig(f'{path}images/plot_ut_time_of_flight.png', dpi=dpi)
    #plt.savefig(f'{path}images/{file.split(".")[0]}.png', dpi=dpi)
except Exception as e:
    print(f"Unable to plot C Scan: {e}")
    pass
plt.close()
