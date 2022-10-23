#!/bin/bash
# update 2022.04.19

## *** Reffer ***
## ・ROSのインストール (Noetic)
##   https://robot.isc.chubu.ac.jp/?p=905
## ・『ROS Noetic』のインストール
##   https://qiita.com/take4eng/items/70f167320ede46e4139c

## **** Reffer 以前(ubuntu18.04+ROS(merodic) ***
## ・Ubuntu18.04: ROS Melodicのインストール
##   https://demura.net/robot/16518.html
## ・ubuntu18.04.3にROSをインストールする
##   https://qiita.com/ezokitsune/items/2e7294b71b869d2909f2

## **** Reffer 以前(ubuntu16.04+ROS(kinetic) ***
## ・Ubuntu install of ROS Melodic
##   http://wiki.ros.org/melodic/Installation/Ubuntu
## ・ROS Melodicのインストール
##   http://joe.ash.jp/program/ros/setup/install_ros_melodic.htm
## ・Getting Started with ROS Melodic on Raspberry Pi 4 Model B
##    https://www.hackster.io/shahizat005/getting-started-with-ros-melodic-on-raspberry-pi-4-model-b-cbdec8

## ubuntu18.04のproxy環境下(会社)などの対策
## https://umashika5555.hatenablog.com/entry/2018/09/25/023919
#$ sudo nano /etc/environment
#-----------------------------------------
#http_proxy="http://proxy-server:port/"
#https_proxy="http://proxy-server:port/"
#-----------------------------------------

## .bashrcの最後に好きなエディタを使い以下の３行を追加する。
## https://demura.net/misc/16564.html
#$ nano ~/.bashrc
#-----------------------------------------
#export ftp_proxy="ftp://プロキシサーバー名:ポート番号/"
#export http_proxy="http://プロキシサーバー名:ポート番号/"
#export https_proxy="https://プロキシサーバー名:ポート番号/"
#-----------------------------------------

## aptの設定を行う。
#$ cd /etc/apt/apt.conf.d
#好きなエディタを使い上のディレクトリ下でapt.confを作成する。ファイルの中身は次のとおり。
#$ nano apt.conf
#-----------------------------------------
#Acquire::http::proxy "http://プロキシサーバー名:ポート番号/";
#Acquire::https::proxy "https://プロキシサーバー名:ポート番号/";
#Acquire::ftp::proxy "ftp://プロキシサーバー名:ポート番号/";
#-----------------------------------------

## If you will see the following error in the terminal:
## E: Could not get lock /var/lib/apt/lists/lock - open (11: Resource temporarily unavailable)
## It can be solved by running following command:
## https://qiita.com/jizo/items/9496496a3156dd39d91a
# sudo rm /var/lib/apt/lists/lock
# sudo rm /var/cache/apt/archives/lock

## 【注意】Ubuntu18.04.2をインストール後、snapdで下記エラーが発生。その場合、コメントアウトして以下の2行を事項
## 【注意】ubuntu-ja-18.04.2-desktop-amd64.iso (Ubuntu18.04.2のJP)でインストールすると発生。素直にUbuntu18.04.3のJPを利用する事
## Ubuntu18.04 error "too early for operation, device not yet seeded or device model not acknowledged"
## https://www.footfoot.tokyo/article104/ubutnu-18-04-too-early-for-operation
#sudo apt -y purge snapd
#sudo apt -y install snapd

## general
echo "## general"
sudo apt -y update
sudo apt -y upgrade
sudo apt -y autoremove

## Setup your sources.list
echo "## Setup your sources.list"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'


## Setup your keys
echo "## Setup your keys"
## apt-key proxy: https://qiita.com/nmatsui/items/816051fe6445db116e9a
#sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --keyserver-option http-proxy=http://x:y@proxy:port --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654


## To be sure that your Ubuntu package index is up to date, type the following command
echo "# To be sure that your Ubuntu package index is up to date, type the following command"
sudo apt -y update
sudo apt -y upgrade

## Install ros-noetic-desktop-full
echo "## ## Install ros-noetic-desktop-full"
sudo apt -y install  ros-noetic-desktop-full

# Environment setup
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

## rosinstall setup
echo "## rosinstall setup"
sudo apt -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
# use catkin build
# refer: https://zenn.dev/suisuiso/articles/08177aacbffc7a
sudo apt -y install python3-osrf-pycommon python3-catkin-tools

## Initialize rosdep
echo "## Initialize rosdep"
sudo rosdep init

## rosdep update
echo "## rosdep update"
rosdep update

## Create and initialize the catkin ws
## https://demura.net/misc/16533.html
echo "## Create and initialize the catkin ws"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init

# catkin build > catkin init
# catkin_make  > catkin_init 

## bulid catkin_ws
echo "## bulid catkin_ws"
cd ~/catkin_ws
catkin build

## Add the catkin_workspace to your ROS environment
echo "## Add the catkin_workspace to your ROS environment"

echo "## Add the catkin_workspace to your ROS environment" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# https://eng-entrance.com/linux-command-echo#echo-3
# echo '' > string
# echo "" > expand value
#echo 'export ROS_PACKAGE_PATH=~/catkin_ws/src:$ROS_PACKAGE_PATH' >> ~/.bashrc
source ~/.bashrc

## Check the ROS environment variables
echo "## Check the ROS environment variables"
export | grep ROS

## Check roscore
echo "## Check roscore"
roscore
