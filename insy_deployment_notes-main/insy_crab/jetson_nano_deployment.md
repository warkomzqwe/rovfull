# Crab ROV Companion Deployment

Posee la siguiente arquitectura de red:
- Companion (Jetson Nano): `192.168.2.2`
- Computador CRABUI: `192.168.2.3`

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

Módulos que corren sobre este computador:

- [Insy Video][insy_video]
- [CRAB Subsystem Middleware][crab_subsystem_middleware]
    - rov_middleware_service
- [CRAB Rov Services][crab_rov_services] 
    - Cam Servo Service
    - Imu Service
    - Depth Service
- [Insy Rov Services][insy_rov_services]
    - Hourmeter
- **Insy Launcher**

### Crontab Line

```bash
@reboot screen -S insy-launcher -dm bash -c "/home/insytech/bin/insy_launcher.sh; exec sh"
```

### Launcher

Crear archivo **insy_launcher.sh** en `~/bin/` y hacerlo ejecutable.

```
touch ~/bin/insy_launcher.sh
chmod +x ~/bin/insy_launcher.sh
nano ~/bin/insy_launcher.sh
```

Agregar:

```bash
#!/bin/bash

_ifconfig=/sbin/ifconfig
insy_bin=/home/insytech/bin

start_screen() {
    screen -S $1 -dm bash -c $2
}

prenetwork_launch() {
    echo "Pre-Network Services..."   
    screen -S gstd -dm bash -c "/usr/local/bin/gstd; exec sh"
}

launch_programs() {
    echo "Launching Insy Software..."
    screen -S video-stream -dm bash -c "$insy_bin/insy-jetson-video; exec sh"
    screen -S rov-middleware -dm bash -c "$insy_bin/rov-middleware-service; exec sh"
    screen -S cam-servo-service -dm bash -c "$insy_bin/cam-servo-service; exec sh"
    screen -S imu-service -dm bash -c "$insy_bin/imu-service; exec sh"
    screen -S depth-service -dm bash -c "$insy_bin/depth-service; exec sh"
    screen -S hourmeter-service -dm bash -c "$insy_bin/hourmeter-service; exec sh"
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

[insy_video]: http://gitlab.telemcloud.cl/aquarovdevelopment/insy-video
[insy_rov_services]: http://gitlab.telemcloud.cl/aquarovdevelopment/insy_rov_services
[static_ip_jetson]: https://forums.developer.nvidia.com/t/static-ip-change-often/186854/5?u=nhasbun
[crab_subsystem_middleware]: http://gitlab.telemcloud.cl/aquarovdevelopment/crab_subsystem_middleware
[crab_rov_services]: http://gitlab.telemcloud.cl/aquarovdevelopment/crab_rov_services
