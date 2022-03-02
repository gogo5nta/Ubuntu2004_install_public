#!/bin/bash
# update 2022.02.07

# --------------------------------------------------------
# NVIDIA Docker って今どうなってるの？ (20.09 版)
# https://medium.com/nvidiajapan/nvidia-docker-%E3%81%A3%E3%81%A6%E4%BB%8A%E3%81%A9%E3%81%86%E3%81%AA%E3%81%A3%E3%81%A6%E3%82%8B%E3%81%AE-20-09-%E7%89%88-558fae883f44
# --------------------------------------------------------
# HP通りやる。ただしPytorch v1.9.0を使用するため★cuda-11-3と指定★
# URL: https://note.com/altbridgetech/n/ne5a320213280
# URL: https://medium.com/@exesse/cuda-10-1-installation-on-ubuntu-18-04-lts-d04f89287130
# URL: https://qiita.com/Navier/items/ec0562e42d8c6e2f504a
# URL: https://qiita.com/Wisteria30/items/6f1e35e600e93ff2c54b
#
# PytorchとCUDAの対応確認
# CUDA 11.1 > pip install torch==1.9.0+cu111 torchvision==0.10.0+cu111 torchaudio==0.9.0 -f https://download.pytorch.org/whl/torch_stable.html
# https://pytorch.org/get-started/previous-versions/
# https://docs.nvidia.com/cuda/cuda-c-programming-guide/index.html#compute-capabilities
# TensorFlowとCUDAの対応関係(# tensorflow-2.4.0  CUDA_11.0  cuDNN_8.0)
# https://www.tensorflow.org/install/source?hl=ja#linux
# https://www.tensorflow.org/install/gpu?hl=ja
#
# apt-key proxy: --keyserver-option http-proxy=http://x:y@proxy:port
# https://qiita.com/nmatsui/items/816051fe6445db116e9a

# https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=20.04&target_type=deb_local
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/11.6.0/local_installers/cuda-repo-ubuntu2004-11-6-local_11.6.0-510.39.01-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2004-11-6-local_11.6.0-510.39.01-1_amd64.deb
sudo apt-key add /var/cuda-repo-ubuntu2004-11-6-local/7fa2af80.pub

sudo apt update
# 自動で合うdriverを入れてくれる
# sudo ubuntu-drivers autoinstallだと465, 以下コマンドだと510のインストールを確認(2022.02.07)
sudo apt -y install cuda-drivers
sudo apt -y --no-install-recommends cuda-11-1
sudo apt -y install cuda-toolkit-11-1

# 以下を.bashrcに追加
# export PATH="/usr/local/cuda/bin:$PATH"
# export LD_LIBRARY_PATH="/usr/local/cuda/lib64:$LD_LIBRARY_PATH"

echo "# CUDA setting" >> ~/.bashrc
echo 'export PATH="/usr/local/cuda/bin:$PATH"' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH="/usr/local/cuda/lib64:$LD_LIBRARY_PATH"' >> ~/.bashrc

# 再起動時画面が黒くなるので、sudo apt update & upgradeを実施
sudo apt update
sudo apt upgrade

# nvidia-driverが正しくインストールしたか再起動で確認
echo "# nvidia-driverが正しくインストールしたか再起動で確認"
echo "$ sudo reboot"
