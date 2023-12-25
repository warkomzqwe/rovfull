sudo -v 
 
# Recommended tools
sudo apt update
sudo apt install nano -y
 
# Installing Python3.8 (Replacing 3.6)
sudo apt install python3-pip python3.8 python3.8-venv python3.8-dev -y
 
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 2
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.6 1
sudo update-alternatives --auto python3
 
 
# Installing imx477 camera drivers for Jetson Nano
# Slight changes are needed for NX
cd ~
pip3 install gdown 
. ~/.profile
 
gdown --id 1vFZSfL7ICEeGa0bQWPy5d34IoCOEdmBz
gdown --id 19OVTOBcZcd-WtkXYYdy2JaEHq-FFDo8I
 
kernel_driver=$(ls nvidia-l4t-kernel_*.deb | head -1)
kernel_driver_dtbs=$(ls nvidia-l4t-kernel-dtbs*.deb | head -1)
sudo apt install --reinstall --allow-downgrades ./$kernel_driver -y &&\
sudo apt install --reinstall --allow-downgrades ./$kernel_driver_dtbs -y
 
## For Jetson Nano A02 use ftd path: /boot/tegra210-p3448-0000-p3449-0000-a02.dtb
 
sudo cp /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.bak
sudo sed -i '/^\s*APPEND.*/a \\      FDT /boot/tegra210-p3448-0000-p3449-0000-b00.dtb' /boot/extlinux/extlinux.conf
 
# Deploying known WiFi connections
sudo nmcli connection add type wifi ifname "" con-name AquarovRD ssid AquarovRD 802-11-wireless-security.key-mgmt WPA-PSK 802-11-wireless-security.psk conce117
