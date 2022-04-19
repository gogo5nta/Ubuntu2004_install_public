#!/bin/bash
# update 2022.04.19

# --- 参考 --------------------------------------------------------------
# ubuntu20.04とROS NoeticでRealSenseを使う手順
# https://qiita.com/porizou1/items/be1eb78015828d43f9fb
# 
# librealsenseのgithub(公式)
# https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
#
# UbuntuでIntel Realsense D415を使えるようにするまで(ROSあり・なし両方)
# https://qiita.com/kei_mo/items/c0387b7d277948451881
# ------------------------------------------------------------------------
# --- 前提 ----------------
# gitがインストール済
# realsense-viewが動作
# ros(noetic)がインストー済

# 参考： rosmakeのために~/rosをROS_PACKAGE_PATHに追加
# https://asukiaaa.blogspot.com/2013/06/rosmakerosrospackagepath.html?m=1
# 確認
# echo $ROS_PACKAGE_PATH 

# .bashrcに追加(追加されてない場合)
#echo "export ROS_PACKAGE_PATH=~/catkin_ws/src:$ROS_PACKAGE_PATH" >> ~/.bashrc
#source ~/.bashrc

# ddynamic_reconfigureのインストール
echo "ddynamic_reconfigureのインストール"
sudo apt install -y ros-noetic-ddynamic-reconfigure

# realsense-rosのセットアップ
echo "realsense-rosのセットアップ"

# realsense-ros ソース入手
echo "realsense-ros ソース入手"
cd ~/catkin_ws/src/
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`

# realsense-ros パッケージのビルド
echo "realsense-ros パッケージのビルド"
cd ~/catkin_ws/
#catkin config --install && catkin clean -y && catkin build
catkin clean -y && catkin build
#catkin build
 
# 参考:新しい「GNOME端末」を開いてコマンドを実行する方法
# https://linuxfan.info/gnome-terminal-with-command

# realsense-ros 動作確認
echo "realsenseを接続"

# realsense2_cameraを起動
echo "realsense2_cameraを起動"
gnome-terminal -- bash -c "roslaunch realsense2_camera rs_camera.launch; bash"

# image_viewを起動
echo "image_viewを起動"
gnome-terminal -- bash -c "rosrun rqt_image_view rqt_image_view; bash"
