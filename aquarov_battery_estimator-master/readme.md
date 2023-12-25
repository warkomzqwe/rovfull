# Battery Estimator

First version of a rough battery __State of Charge__ (_SoC_) intended to use 
with 18650 Lithium Ion Battery cells.

Estimate is done using __Open Circuit Voltage__ (_OCV_) values and 
top/bottom expected voltages to define full/empty battery SoC. To accomplish
this, raw measurements of current and battery voltage are used. This way we can 
discard voltage measurements when battery is with a high load, working then with
stables values near expected OCV. 

## Quick Install

Using deploy tokens: 

```bash
pip install git+http://gitlab+deploy-token-20:g2VrU8rS18n5BTbm1PDz@gitlab.telemcloud.cl/aquarovdevelopment/aquarov_battery_estimator.git
```

* **Python 3.6+** compatible library.

### Jetson Nano Installation

1. Default user **nvidia**, password **nvidia**.

1. Add user to relevant groups **i2c, dialout and tty**.

1. Setup WiFi/Internet using a USB WiFi dongle - [Setup for WPA2 networks](https://superuser.com/a/477393)

1. Install **python3-venv**, clone repository, create virtual environment, activate then install service:

```bash
sudo -v

sudo usermod -a -G i2c $USER 
sudo usermod -a -G dialout $USER 
sudo usermod -a -G tty $USER

cd $HOME

sudo apt install python3-venv python3-dev -y
git clone http://gitlab+deploy-token-20:g2VrU8rS18n5BTbm1PDz@gitlab.telemcloud.cl/aquarovdevelopment/aquarov_battery_estimator.git && \
cd aquarov_battery_estimator && python3 -m venv venv && . ./venv/bin/activate && \
pip install -U pip setuptools wheel && \
pip install -e .

# Registering executables
mkdir -p $HOME/bin

ln -s $PWD/venv/bin/battery_estimator_service $HOME/bin/battery-estimator

deactivate
cd $HOME

```

## Quick Usage:

Associated commands,

* `battery_estimator_service`: Battery SoC report via LAN.

### Service Auto-start

Using **crontab** and **screen**:

1. Edit crontab file with `crontab -e`


1. Add new job to crontab (:warning: Python virtual environment path depends on your system installation.)
```bash
@reboot sleep 1 && screen -S battery_estimator -dm bash -c ". /home/nvidia/aquarov_battery_estimator/venv/bin/activate && battery_estimator_service; echo ending; exec sh"
```
