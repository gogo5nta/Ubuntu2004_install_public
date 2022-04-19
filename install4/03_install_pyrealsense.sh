#!/bin/bash
# update 2022.04.19


# 不足しているライブラリをpipでインストール。requirements.txtを参照
#　参考：https://qiita.com/sakusaku12/items/21083c73c8afa4f6c78d
#　　　　https://note.nkmk.me/python-pip-install-requirements/

# python install
sudo apt -y install python python3-pip

# pip update
pip install --upgrade pip setuptools

# pipで一括環境構築
pip install -r requirements.txt
