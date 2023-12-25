#!/bin/bash

gst_dependencies="
    gstreamer1.0-plugins-base
    gstreamer1.0-plugins-bad
    gstreamer1.0-plugins-ugly
    gstreamer1.0-plugins-good
    libcairo2-dev
    libgirepository1.0-dev
    libopenjp2-7
    python3-dev
    python3-venv
    ffmpeg
    libgirepository1.0-dev
    python3-gst-1.0
    gstreamer1.0-tools
    libjpeg-dev
    libxslt1-dev"

gst_interpipe_dep="
    libgstreamer1.0-dev
    libgstreamer-plugins-base1.0-dev
    gtk-doc-tools"

gstd_dependencies="
    automake
    libtool
    pkg-config
    libgstreamer1.0-dev
    libgstreamer-plugins-base1.0-dev
    libglib2.0-dev
    libjson-glib-dev
    gtk-doc-tools
    libreadline-dev
    libncursesw5-dev
    libdaemon-dev
    libjansson-dev
    libsoup2.4-dev
    libedit-dev
    python3-pip"

recommended_deps="
    git
    screen
    v4l-utils"

install_package () {
    sudo apt install $1 -y
}

print_banner () {
    echo ""
    echo "*********************************************************"
    echo $1
    echo $2
    echo "*********************************************************"
    echo ""
}

# Request for sudo at start
sudo -v && sudo apt update

# Installing special dependencies
install_package apt-utils
echo ttf-mscorefonts-installer msttcorefonts/accepted-mscorefonts-eula select true | sudo debconf-set-selections
install_package ttf-mscorefonts-installer

all_dependencies="$gst_dependencies $gst_interpipe_dep $gstd_dependencies $recommended_deps"
print_banner "Installing dependencies:  " "$all_dependencies"

# Install all declared dependencies
install_package "$all_dependencies"
echo "Installed packages code: $?"

# Avoiding «server certificate verification failed» error
export GIT_SSL_NO_VERIFY=1

# Preparing directories
mkdir -p $HOME/.local/share/rov-gcs-system/{src,scripts,logs} 
mkdir -p $HOME/.local/bin 
mkdir -p $HOME/rov_media/public/rov_{recordings,snapshots}

# Installing GstInterpipe
cd $HOME/.local/share/rov-gcs-system/src
git clone https://github.com/RidgeRun/gst-interpipe.git
cd $HOME/.local/share/rov-gcs-system/src/gst-interpipe
./autogen.sh --libdir /usr/lib/aarch64-linux-gnu/
make
sudo make install

# Installing GstD
cd $HOME/.local/share/rov-gcs-system/src
git clone --depth 1 --branch v0.15.0 https://github.com/RidgeRun/gstd-1.x.git
cd $HOME/.local/share/rov-gcs-system/src/gstd-1.x
./autogen.sh
./configure
make
sudo make install

# Install internal rov services and rov video repos
cd $HOME
git clone http://gitlab+deploy-token-19:zB72ekAWbUztgxpyLzzG@gitlab.telemcloud.cl/aquarovdevelopment/insy-video.git insy_video
cd insy_video
python3 -m venv venv
. ./venv/bin/activate
pip install -U pip wheel setuptools
pip install -e .
deactivate

# Register executables
mkdir -p $HOME/bin
ln -s $PWD/venv/bin/insy-jetson-video $HOME/bin/insy-jetson-video

cd $HOME
