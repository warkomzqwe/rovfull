# Insy 308 Jetson Nano Deployment

Posee la siguiente arquitectura de red:
- Companion (Jetson Nano): `192.168.2.2`

## Sistema Base

Requisitos previos a despliegue:

- Jetson Nano con Jetpack 4.5.1
- [IP estática][static_ip_jetson] `192.168.2.2`
- Conexión a vpn **Atenas**
- Intérprete **Python 3.8**

```bash
# File /etc/network/interfaces.d/eth0
 
auto eth0
iface eth0 inet static
  address 192.168.2.2
  netmask 255.255.255.0
```

### Despliegue

Drivers extra requeridos:
- [IMX477 Camera Driver][imx477_drivers]
    - Ver guía interna: [IMX477 Driver Installation](../common/imx477_driver_installation.md)

Modulos que corren sobre este computador:

- **Insy Launcher**
- [Insy Video][insy_video]
- [Rov Multiserver][rov_multiserver]
- [Insy Rov Services][insy_rov_services]
    - Hourmeter
    - Arduino Acamas Bridge
- [Aquarov Battery Estimator][battery_estimator]
- [Insy Power Sensor Reader][insy_power_sensor_reader]
<!-- - [Insytech Power Manager Bridge][power_manager_board]
- [Thruster Power Sense][thruster_power_sense] -->
- **Insy DHCP Server**

### DHCP Server Notes

```
sudo apt install isc-dhcp-server -y
 
# Edit config file, copy example below
sudo mv /etc/dhcp/dhcpd.conf{,.backup}
sudo nano /etc/dhcp/dhcpd.conf
 
# Edit interfaces file, copy example below
sudo nano /etc/default/isc-dhcp-server
 
sudo systemctl restart isc-dhcp-server.service
sudo systemctl enable isc-dhcp-server.service
```

```
# file /etc/dhcp/dhcpd.conf
default-lease-time 600;
max-lease-time 7200;
authoritative;
 
subnet 192.168.2.0 netmask 255.255.255.0 {
 range 192.168.2.100 192.168.2.200;
 option routers 192.168.2.1;
 option domain-name-servers 8.8.8.8, 8.8.4.4;
}
```

```
# file /etc/default/isc-dhcp-server
INTERFACESv4="eth0"
INTERFACESv6=""
```

### Root crontab

```bash
@reboot screen -S vpn-status -dm bash -c ". /root/insytech_vpn_client/vpn_status.sh; exec sh"
@reboot screen -S vpn-starter -dm bash -c ". /root/insytech_vpn_client/vpn_start.sh; exec sh"
@reboot screen -S http-server -dm bash -c "cd /home/insytech/rov_media/public && python3 -m http.server 80"
```

### User crontab

```bash
@reboot screen -S insy-launcher -dm bash -c ". /home/insytech/bin/insy_launcher.sh; exec sh"
```

### Launcher

```bash
#!/bin/bash

_ifconfig=/sbin/ifconfig
insy_bin=/home/insytech/bin

prenetwork_launch() {
    echo "Pre-Network Services..."   
    screen -S gstd -dm bash -c "/usr/local/bin/gstd; exec sh"
}

launch_programs() {
    echo "Launching Insy Software..."
    screen -S video-stream -dm bash -c "$insy_bin/insy-jetson-video; exec sh"
    screen -S rov-navigation -dm bash -c "$insy_bin/rov-navigation; exec sh"
    screen -S acamas-bridge -dm bash -c "$insy_bin/arduino-acamas-bridge; exec sh"
    screen -S battery-estimator -dm bash -c "$insy_bin/battery-estimator; exec sh"
    screen -S hourmeter -dm bash -c "$insy_bin/hourmeter-service; exec sh"
    screen -S ftp-service -dm bash -c "$insy_bin/ftp-service; exec sh"
    screen -S power-sensor-reader -dm bash -c "$insy_bin/power-sensor-reader -v; exec sh"
}

prenetwork_launch

while true
do
   $_ifconfig eth0 | grep "inet 192.168.2.2" > /dev/null
   if [ $? -gt 0 ]
   then
        echo "Interface eth0 not running"           
        sleep 2
   else
        launch_programs
        break
   fi
done
```



## Bitácora

- 8 de agosto de 2022:
    - Faltan instrucciones para instalar mavproxy en el sistema
    - mavproxy necesita deshabilitar ModemManager de Ubuntu
    - Falta agregar ardusub_uploader a herramientas y requisito pyserial
    - Falta agregar micro-servicio para dhcp server (se utiliza node-dhcp)
    
    ```bash
    # Install nodejs using node version manager
    sudo -v

    sudo su
    cd ~
    wget -qO- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash
    
    exit
    sudo su
    cd ~
    nvm install --lts

    ```

- 9 de agosto de 2022:
    - Crear carpeta para archivos de configuración es altamente recomendable

- 23 de agosto de 2022:
    - Recomendable dejar uploader como un binario empaquetado con pyserial

- 13 de septiembre de 2022:
    - Principal impedimento para dejar todo automático es el deployment de dhcp
        - [Run multiple commands as sudo](https://www.cyberciti.biz/faq/how-to-run-multiple-commands-in-sudo-under-linux-or-unix/)
    - Otro impedimento es agregar wifi conocido por cli
        - `sudo nmcli connection add type wifi ifname "" con-name AquarovRD ssid AquarovRD 802-11-wireless-security.key-mgmt WPA-PSK 802-11-wireless-security.psk conce117`


[insy_video]: http://gitlab.telemcloud.cl/aquarovdevelopment/insy-video
[rov_multiserver]: http://gitlab.telemcloud.cl/aquarovdevelopment/rov-multiserver
[insy_rov_services]: http://gitlab.telemcloud.cl/aquarovdevelopment/insy_rov_services
[battery_estimator]: http://gitlab.telemcloud.cl/aquarovdevelopment/aquarov_battery_estimator
[power_manager_board]: http://gitlab.telemcloud.cl/aquarovdevelopment/insy_power_manager_firmware
[thruster_power_sense]: http://gitlab.telemcloud.cl/aquarovdevelopment/thruster_power_sense
[imx477_drivers]: https://developer.ridgerun.com/wiki/index.php/Raspberry_Pi_HQ_camera_IMX477_Linux_driver_for_Jetson
[static_ip_jetson]: https://forums.developer.nvidia.com/t/static-ip-change-often/186854/5?u=nhasbun
[insy_power_sensor_reader]: http://gitlab.telemcloud.cl/aquarovdevelopment/insypowersensorreader