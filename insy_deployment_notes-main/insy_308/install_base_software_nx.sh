sudo -v 

initial_path=$PWD
 
# Recommended tools
sudo apt update
sudo apt install nano -y
 
source ../common/config_python3.sh
source ../common/install_mavproxy.sh 
source ../common/config_wifi.sh
 
# Installing imx477 camera drivers for Jetson Nx
cd ~
pip3 install gdown 
. ~/.profile
 
gdown 11zNF1pViu1U0nGNLeK7Ivw0L3zn-fjtu
 
kernel_driver_dtbs=$(ls nvidia-l4t-kernel-dtbs*.deb | head -1) 
sudo apt install --reinstall --allow-downgrades ./$kernel_driver_dtbs -y

# Improving imx477 image using ISP file
# According to https://forums.developer.nvidia.com/t/raspberry-pi-hq-camera-in-jetson-xavier-nx/142761/18
sudo rm /var/nvidia/nvcam/settings/nvcam_cache_*
sudo rm /var/nvidia/nvcam/settings/serial_no_*

sudo cp $initial_path/camera_overrides.isp /var/nvidia/nvcam/settings

sudo chmod 664 /var/nvidia/nvcam/settings/camera_overrides.isp
sudo chown root:root /var/nvidia/nvcam/settings/camera_overrides.isp

 
## For Jetson Nano A02 use ftd path: /boot/tegra210-p3448-0000-p3449-0000-a02.dtb
 
sudo cp /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.bak
sudo sed -i '/^\s*APPEND.*/a \\      FDT /boot/tegra194-p3668-all-p3509-0000.dtb' /boot/extlinux/extlinux.conf
 
