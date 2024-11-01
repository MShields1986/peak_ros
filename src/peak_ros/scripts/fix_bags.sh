#!/usr/bin/env bash
source /home/matthew/.bashrc
source /home/matthew/Desktop/phd_workspaces/kuka_kmr_driver/catkin_ws/devel/setup.bash

shopt -s nullglob

rm fixed_*

arr=(../bags/*.bag)
pattern="peak_recording"
replacement="fixed"

for f in "${arr[@]}"; do
    # https://github.com/gavanderhoorn/rosbag_fixer.git
    python3 /home/matthew/Desktop/torque_data/rosbag_fixer/fix_bag_msg_def.py -l "$f" "${f/"$pattern"/"$replacement"}"
    rosbag reindex "${f/"$pattern"/"$replacement"}"
done

rm *.orig.bag
