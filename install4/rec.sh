#!/bin/bash
# Ubuntu20.04.4 + ROS(melodic)
# update 2022.04.19

echo "start rec realsense D435i"
echo "END: windows上で[esc]"
python3 realsense_recorder_D435i.py --record_imgs --output_folder ./data

# ./dataに以下が生成される
#　　・rgbフォルダ
#　　・depthフォルダ
#　　・rgb.txt
#　　・depth.txt
#　　・camera_intrinsic.json
