#!/usr/bin/env bash

com_port=$(ls /dev/serial/by-id/usb-ArduPilot* | head -1)
/home/$USER/.local/bin/mavproxy.py --master=$com_port,115200 --load-module='GPSInput,DepthOutput' --source-system=200 --cmd="set heartbeat 0" --out udpin:localhost:9000 --out udpout:localhost:9002 --out udpin:0.0.0.0:14660 --out udpbcast:192.168.2.255:14550 --out udpbcast:10.24.12.255:14550 --mav20 --aircraft telemetry --streamrate 10
