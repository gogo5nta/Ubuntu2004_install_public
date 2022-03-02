#!/bin/bash
# update 2021.04.24

# --------------------------------------------------------
# Ubuntu18.04にNVIDIA Container Toolkitをインストールする
# https://qiita.com/Wisteria30/items/6f1e35e600e93ff2c54b
# --------------------------------------------------------
# 事前にnvidia-driverをインストールし、rebootする
# 
# apt-key proxy: --keyserver-option http-proxy=http://x:y@proxy:port
# https://qiita.com/nmatsui/items/816051fe6445db116e9a
#
# curl proxy: -x http://x:y@proxy:port
# https://qiita.com/tkj/items/c6dad4efc0dff4fecd93

# 確認(nvidia driver)
nvidia-smi

# 確認(nvidia cuda)
nvcc -V

# Dockerをインストール
sudo apt update
sudo apt install apt-transport-https ca-certificates curl gnupg-agent software-properties-common -y
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo apt-key fingerprint 0EBFCD8
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io -y

# 確認(Docker)
sudo docker run hello-world

# 確認(Docker version)
docker -v


# --------------------------------------------------------
# NVIDIA container toolkitを使って、dockerのコンテナ上でcudaを動かす
# URL:https://qiita.com/Hiroaki-K4/items/c1be8adba18b9f0b4cef
# --------------------------------------------------------
# 環境 (2021.05.05)
# Ubuntu 18.04
# docker 20.10.6
# nvidia driver 465.19

# Add the package repositories
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

# 確認(docker --gpus)
# sudo docker run --gpus all nvidia/cuda:10.1-base-ubuntu18.04 nvidia-smi
# sudo docker run --gpus all nvidia/cuda:11.0-base-ubuntu18.04 nvidia-smi
sudo docker run --gpus all nvidia/cuda:11.0-base-ubuntu20.04 nvidia-smi

# Dockerコマンドをsudoなしで実行する方法
# URL:https://insilico-notebook.com/docker-run-without-sudo/
# URL:https://qiita.com/DQNEO/items/da5df074c48b012152ee
# dockerグループがなければ作る
sudo groupadd docker

# 現行ユーザをdockerグループに所属させる
sudo usermod -aG docker $USER 
newgrp docker 

# dockerデーモンを再起動する (CentOS7の場合)
sudo systemctl restart docker

# test
docker run hello-world 

# docker-composeのインストール
# URL:https://qiita.com/hgoj/items/1ba050c7d73e0dd5ef90
# URL:https://docs.docker.jp/compose/install.html#linux　　←古い
 
sudo curl -L https://github.com/docker/compose/releases/download/1.29.2/docker-compose-`uname -s`-`uname -m` -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose

# docker-composeの確認
docker-compose -v
