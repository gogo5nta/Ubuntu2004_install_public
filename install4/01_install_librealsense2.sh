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

# 公開鍵を登録
echo "公開鍵を登録"

#公開鍵でおかしくなった場合、librelasenseのgithubで再度確認
#https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
#一般：
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

#Proxy内(会社)
#export http_proxy="http://<proxy>:<port>"
#sudo -E apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

# 注意:リポジトリサーバーはamazon > intelへ
# オリジナルサイトを参照: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
# サーバーをレポジトリリストに登録
echo "サーバーをレポジトリリストに登録"
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

# ライブラリ(liblealsense2)をインストール
echo "ライブラリ(liblealsense2)をインストール"
sudo apt-get -y install librealsense2-dkms
sudo apt-get -y install librealsense2-utils

#開発者用ツール、デバッガツールをインストール(realsense-rosをインストールする場合、必須)
sudo apt-get -y install librealsense2-dev
sudo apt-get -y install librealsense2-dbg


#パッケージのアップグレード
#　参考：Ubuntu18.04: RealSense D435iをROS Melodicで使う
#　　　　https://demura.net/robot/16525.html
#
#以下の手順でアップグレードする。ここでは上でインストールしたパッケージだけアップグレードする。
#アップグレード可能なすべてのパッケージをアップグレードすると稀に動かなくなるパッケージがある
#のでこの方法はお勧め。

echo "パッケージのアップグレード"
sudo apt update
#ライブラリ
sudo apt --only-upgrade -y install librealsense2-utils librealsense2-dkms

#ライブラリ+デバッグ(オプション)
sudo apt --only-upgrade -y install librealsense2-utils librealsense2-dkms librealsense2-dev librealsense2-dbg 


#PCを一度再起動する事
echo "PCを一度再起動する事"
echo "sudo reboot"


#動作確認(realsenseをつなげ、以下のコマンドによりrealsense viewerを立ち上げ)
#reslsense-viewer
echo "動作確認(realsenseをつなげ、以下のコマンドによりrealsense viewerを立ち上げ)"
echo "reslsense-viewer"
