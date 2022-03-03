#!/bin/bash

# install cuda 11.3
if command -v nvcc &>/dev/null; then echo cuda is already installed; exit 0; fi

wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin || exit 1
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/11.3.0/local_installers/cuda-repo-ubuntu2004-11-3-local_11.3.0-465.19.01-1_amd64.deb || exit 1
sudo dpkg -i cuda-repo-ubuntu2004-11-3-local_11.3.0-465.19.01-1_amd64.deb || exit 1
sudo apt-key add /var/cuda-repo-ubuntu2004-11-3-local/7fa2af80.pub || exit 1
sudo apt-get update
sudo apt-get -y install cuda || exit 1

# pytorch install (für cuda 11.3)
pip3 install torch==1.10.2+cu113 torchvision==0.11.3+cu113 torchaudio==0.10.2+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html || exit 1

AuxMePW3065
