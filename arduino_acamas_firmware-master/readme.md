# Insytech Firmware for Arduino Acamas

Robotic node control for:

* Bluerobotics Grabber
* ROV Lights (x3)

## Firmware Upload

Compile firmware and upload with platormio-cli:

```bash
pio run -e nanoatmega328new -t upload --upload-port /dev/ttyDummyS0
```

List available USB devices:
```bash
pio device list
```