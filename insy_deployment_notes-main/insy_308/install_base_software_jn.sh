sudo -v 
 
# Recommended tools
sudo apt update
sudo apt install nano -y

source ../common/config_python3.sh
source ../common/install_mavproxy.sh
source ../common/config_wifi.sh 

# Installing imx477 camera drivers for Jetson Nano
# Slight changes are needed for NX
cd ~
pip3 install gdown 
. ~/.profile
 
gdown 1vFZSfL7ICEeGa0bQWPy5d34IoCOEdmBz
 
kernel_driver_dtbs=$(ls nvidia-l4t-kernel-dtbs*.deb | head -1)
sudo apt install --reinstall --allow-downgrades ./$kernel_driver_dtbs -y
 
## For Jetson Nano A02 use ftd path: /boot/tegra210-p3448-0000-p3449-0000-a02.dtb
 
sudo cp /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.bak
sudo sed -i '/^\s*APPEND.*/a \\      FDT /boot/tegra210-p3448-0000-p3449-0000-b00.dtb' /boot/extlinux/extlinux.conf
 

