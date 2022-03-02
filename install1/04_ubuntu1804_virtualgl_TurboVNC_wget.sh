#!/bin/bash
# update 2021.05.05

echo "文字化け対策 .bashrcの末端に記入"
echo '# 文字化け対策 .bashrcの末端に記入' >> ~/.bashrc
echo 'case $TERM in' >> ~/.bashrc
echo '      linux) LANG=C ;;' >> ~/.bashrc
echo '      *)       LANG=ja_JP.UTF-8;;' >> ~/.bashrc
echo 'esac' >> ~/.bashrc

# -----------------------------------------------------------------------------
# Ubuntu18.04にvirtualglとTurbovncをインストール
# URL1: https://qiita.com/exthnet/items/dcb0bd94f09a2b4c9835
# URL2: https://qiita.com/Shunmo17/items/c2e21af5d5f609c01b08
# URL3: https://geraniums.hatenablog.com/entry/2018/05/25/151153
# -----------------------------------------------------------------------------
echo "#wget virtualgl_2.6.5_amd64.deb"
wget https://sourceforge.net/projects/virtualgl/files/2.6.5/virtualgl_2.6.5_amd64.deb/download
mv download virtualgl_2.6.5_amd64.deb

echo "#wget turbovnc_2.2.6_amd64.deb"
wget https://sourceforge.net/projects/turbovnc/files/2.2.6/turbovnc_2.2.6_amd64.deb/download
mv download turbovnc_2.2.6_amd64.deb

echo "# install virtualgl"
sudo dpkg -i virtualgl_*_amd64.deb

# -----------------------------------------------------------------------------
# *** ここからはCUIで実施 ***
# -----------------------------------------------------------------------------
# echo "stop desktop"
# --- ubuntu 16.04 ---
# echo "Ctl + Alt + F1でCUIに移動。事前に文字化け対策必要"
# $ sudo systemctl stop lightdm

# --- ubuntu 18.04 or 20.04 ---
# echo "Ctl + Alt + F3でCUIに移動。事前に文字化け対策必要"
# $ sudo systemctl stop gdm

# echo "setup virtualgl"
# echo "途中の質問は右を参考 https://virtualgl.org/vgldoc/2_2_1/#hd005001"
# echo "1, y, n, n, n, x "
# $ sudo /opt/VirtualGL/bin/vglserver_config

# echo "start desktop"
# --- ubuntu 16.04 ---
# $ sudo systemctl start lightdm

# --- ubuntu 18.04  or 20.04 ---
# $ sudo systemctl start gdm

# echo "install turboVNC"
# $ sudo dpkg -i turbovnc_*_amd64.deb
 
# echo "start TurboVNC"
# echo "初回はパスワード登録。パスワード文字列長が8に制限されているのに注意"
# $ /opt/TurboVNC/bin/vncserver -depth 24

# echo "起動中のVNCを確認"
# echo "参考:https://qiita.com/Sawahashi/items/156be0baaf6384884f3d"
# $ /opt/TurboVNC/bin/vncserver -list

# echo "stop TurboVNC"
# $ /opt/TurboVNC/bin/vncserver -kill :1

## windowsでTurboVNCによりログインする時
## https://sourceforge.net/projects/turbovnc/files/2.2.6/
## 上記URLからTurboVNC-2.2.6-x64.exeをダウンロード＆インストール＆TurboVNCを起動
## ubuntu側の/home/username/.vnc/xxx.logにポート番号を記載(例:5901など)
## 例： 192.168.0.44:5902
## 例： 192.168.0.44:2

## ★注意★ vncをログインして黒画面が出たら(anacondaインストール後)
## この場合は Anaconda が悪さをしているようだが、Anaconda を使っていない場合でも同様な問題が起こることがある。
## http://penguinitis.g1.xrea.com/computer/linux/VNC.html
##　↓
## Anacondaをインストール後、.bashrcの以下がコメントしているかチェック
## export PATH="/root/anaconda3/bin:$PATH"

## --- ここからは必要なら実施 ---
## turbovncの起動エイリアスを作成. nano ~/.bashrcを実行し
## alias TVNC='/opt/TurboVNC/bin/vncserver -depth 24' 

## windowsのおすすめsshソフト
## rlogin    > https://qiita.com/pocket8137/items/e294715b5154487b9ae0
## MobaXterm > https://qiita.com/mkasahara/items/c029154b5436913b20e0
