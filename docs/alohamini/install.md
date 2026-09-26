# Installation & Configuration

Applies to all robots and all machines (PC and Raspberry Pi).

## Install conda

**x86-64 (PC / Ubuntu):**

```bash
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm ~/miniconda3/miniconda.sh
~/miniconda3/bin/conda init bash
source ~/.bashrc
```

**ARM64 (Raspberry Pi):**

```bash
wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh \
  -O ~/miniforge3/miniforge.sh
bash ~/miniforge3/miniforge.sh -b -u -p ~/miniforge3
rm ~/miniforge3/miniforge.sh
~/miniforge3/bin/conda init bash
source ~/.bashrc
```

## Clone and install

```bash
git clone https://github.com/liyiteng/lerobot_alohamini.git
cd lerobot_alohamini
conda create -y -n lerobot_alohamini python=3.12
conda activate lerobot_alohamini
pip install -e ".[all]"
pip install pyzmq feetech-servo-sdk
conda install -y ffmpeg=7.1.1 -c conda-forge
```

## Serial port permissions

One-time setup — add your user to the `dialout` group, then reboot:

```bash
sudo usermod -a -G dialout $USER
# reboot for the change to take effect
```

## HuggingFace configuration

Create an account at [huggingface.co](https://huggingface.co) and generate a token with read + write permissions, then log in:

```bash
git config --global credential.helper store
hf auth login --token <your_token> --add-to-git-credential
```

Get your username (used in dataset paths throughout this guide):

```bash
HF_USER=$(hf auth whoami | sed 's/^user=//')
echo $HF_USER
```
