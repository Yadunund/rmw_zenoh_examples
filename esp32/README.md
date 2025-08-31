## Zenoh Pico on ESP32 Microcontrollers

## Setup
- Ensure your computer has the necessary drivers installed to transfer data to your ESP32 microcontroller.
  - For example, if your microcontroller relies on a CH343 chip, ensure you have [this driver module](https://github.com/WCHSoftGroup/ch343ser_linux) installed.
- Install [PlatformIO](https://docs.platformio.org/en/latest/core/installation/methods/installer-script.html)
- Initialize git submodules to download dependencies:
```bash
git submodule update --init --recursive
```
- Build [zenoh-pico](https://github.com/eclipse-zenoh/zenoh-pico) examples locally
```bash
git clone https://github.com/eclipse-zenoh/zenoh-pico -b 1.4.0
cd zenoh-pico && make
```

## Upload code to the board
The esp32 is programmed to publish `nav_msgs/msg/Odometry` at 50hz.

To compile and upload the program to the esp32 microcontroller, first ensure the connected board is properly configured in [platformio.ini](./platformio.ini).

Then run
```bash
cd esp32
platformio run
platformio run -t upload
```

By default the board will try to connect to a previously connected WiFi access point.
When this fails (example for the first time), it will go into AP mode with SSID `ZenohESP32`.
From a phone or computer, connect to this SSID with default password `zenoh123`.
Open the WiFi login page (192.168.4.1) and click `Configure WiFi`.
Connect to an access point and also set the `Zenoh locator` to the endpoint of the Zenoh router running on your computer.

## Demo

On your computer, start the Zenoh router

```bash
source /opt/ros/jazzy/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

You can then subscribe to messages published by the ESP32 using:

```bash
ros2 topic echo /odom nav_msgs/msg/Odometry
```

The subscriber should receive messages published by the microcontroller.
