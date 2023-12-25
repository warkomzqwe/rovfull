# Installing Python3.8 (Replacing 3.6)
sudo apt install python3-pip python3.8 python3.8-venv python3.8-dev -y
 
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 2
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.6 1
sudo update-alternatives --auto python3
 