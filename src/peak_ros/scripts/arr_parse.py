#!/usr/bin/env python3

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable

path = "/home/matthew/Desktop/phd_workspaces/kuka_kmr_driver/catkin_ws/src/peak_ros/src/peak_ros/bags/"

#file = "pp_full_1745422377521490.pcd"
#file = "pp_full_1745422390265942.pcd"
#file = "pp_full_1745422641176811.pcd"
#file = "pp_full_1745423618327693.pcd"
#file = "pp_full_1745425047850435.pcd"
file = "pp_full_1745425284821404.pcd"


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

data = data.to_numpy(np.float32)

data[:,3] = (data[:,3]- 0.5) * 2

x_unique = np.unique(data[:, 0])
y_unique = np.unique(data[:, 1])
z_unique = np.unique(data[:, 2])

x_to_idx = {v: i for i, v in enumerate(x_unique)}
y_to_idx = {v: i for i, v in enumerate(y_unique)}
z_to_idx = {v: i for i, v in enumerate(z_unique)}

volume = np.full((len(x_unique), len(y_unique), len(z_unique)), np.nan)

for x, y, z, amp in data:
    xi = x_to_idx[x]
    yi = y_to_idx[y]
    zi = z_to_idx[z]
    volume[xi, yi, zi] = amp

volume_c = abs(volume)
b_scan_index = 70

# B-scan plot
plt.figure(dpi=300, figsize=(8, 5))
plt.imshow(volume[b_scan_index, :, :].T, aspect='auto', vmin=-1, vmax=1)
plt.title("B-scan at scanning step X")
plt.ylabel("Depth - Time samples [-]")
plt.xlabel("Array axis [-]")
cbar = plt.colorbar()
cbar.set_label("Normalised amplitude [-]")
plt.tight_layout()
plt.show()

# A-scan plot
plt.figure(dpi=300, figsize=(6, 4))
plt.plot(volume[b_scan_index, 19, :])
plt.title("A-scan at scanning step XXX, array element YYY")
plt.xlabel("Depth - Time samples [-]")
plt.ylabel("Normalised amplitude [-]")
plt.xlim(0, 1450)
plt.ylim(-1,1)
plt.grid(True, linestyle='--', alpha=0.7)
plt.tight_layout()
plt.show()


adjusted_index = b_scan_index

# C-scan plot
c_scan = np.argmax(volume_c[:, :, 600:1100], axis=2)

fig, ax = plt.subplots(dpi=300, figsize=(7, 6))
im = ax.imshow(c_scan[20:].T, cmap='rainbow', origin='lower')
ax.set_title("C-scan Time-of-Flight")
ax.set_xlabel("Scanning axis [-]")
ax.set_ylabel("Array axis [-]")
divider = make_axes_locatable(ax)
cax = divider.append_axes("right", size="5%", pad=0.05)
cbar = plt.colorbar(im, cax=cax)
cbar.set_label("Time-of-flight \n (time sample)")
ax.axvline(adjusted_index, color='#39ff14', linestyle='--', linewidth=1, label='B-scan location (X=70)')
ax.legend(loc='upper right')
plt.tight_layout()
plt.show()
