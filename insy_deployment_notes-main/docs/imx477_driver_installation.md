# Instrucciones para la instalación de los drivers necesarios para la cámara **Raspberry Pi HQ camera IMX477**.

[[_TOC_]]

# Referencias

* [Raspberry Pi HQ camera IMX477 Linux driver for Jetson](https://developer.ridgerun.com/wiki/index.php?title=Raspberry_Pi_HQ_camera_IMX477_Linux_driver_for_Jetson)
* [Jetson Nano B01 vs A02: What’s New for the Compute on Module (CoM) and Carrier Board](https://www.arducam.com/nvidia-jetson-nano-b01-update-dual-camera/)

# Jetson Nano 4GB

## Preparación

Previo a la instalación se debe definir si la placa es la versión **A02** o **B01**.

<img src="./img/nvidia-jetson-nano-a02-pinout.jpg" width="400"><br>
_Version A02_

<img src="./img/nvidia-jetson-nano-b01-pinout.jpg" width="400"><br>
_Version B01_


Versiones de Jetpack compatibles:
*  Jetpack 4.4
*  Jetpack 4.4.1
*  Jetpack 4.5

Se debe elegir el driver correspondiente desde la siguiente [carpeta de google drive](https://drive.google.com/drive/folders/17SJ0NNSi0xiL7w8lVUtNzvAZ8Sm3Oggu).

## Carga de Driver

Copiar los drivers a la Jetson Nano y ejecutar:

```bash
sudo -v
kernel_driver=$(ls nvidia-l4t-kernel_*.deb | head -1)
kernel_driver_dtbs=$(ls nvidia-l4t-kernel-dtbs*.deb | head -1)
sudo apt install --reinstall --allow-downgrades ./$kernel_driver -y &&\
sudo apt install --reinstall --allow-downgrades ./$kernel_driver_dtbs -y
:


```

Editar el archivo **/boot/extlinux/extlinux.conf** de forma automática y reboot (válido para **Jetson Nano B01**):
```bash
# For Jetson Nano A02 use ftd path: /boot/tegra210-p3448-0000-p3449-0000-a02.dtb

sudo cp /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.bak
sudo sed -i '/^\s*APPEND.*/a \\      FDT /boot/tegra210-p3448-0000-p3449-0000-b00.dtb' /boot/extlinux/extlinux.conf
sudo reboot


```

# Jetson Nano 2GB

Hasta el momento ha funcionado bien tratar esta placa como si fuera una **B01**.

# Quick Test

Cámara debería aparecer ahora al ejecutar:
```bash
ls /dev/video* && v4l2-ctl --list-formats-ext
```

Respuesta esperada:
```bash
/dev/video0
ioctl: VIDIOC_ENUM_FMT
        Index       : 0
        Type        : Video Capture
        Pixel Format: 'RG10'
        Name        : 10-bit Bayer RGRG/GBGB
                Size: Discrete 4032x3040
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 1920x1080
                        Interval: Discrete 0.017s (60.000 fps)
```


