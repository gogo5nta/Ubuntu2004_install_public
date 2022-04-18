#!/bin/bash
# update 2022.04.18

# --------------------------------------------------------
# Ubuntu18.04にNVIDIA Container Toolkitをインストールする
# https://qiita.com/Wisteria30/items/6f1e35e600e93ff2c54b
# --------------------------------------------------------

# 古いdockerを削除
# sudo apt-get remove docker docker-engine docker.io containerd runc

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
