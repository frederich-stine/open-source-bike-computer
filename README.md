# Open-Source Bike Computer

## Project Description

Open-Source Bike Computer project for EN.525.743 JHU EP SP 2023

This project creates a usable bike computer with external speed and cadence sensors.

Features:

- GPS
- Temperature Sensing
- Speed
- Cadence
- Display

This project was written for Seeed studio hardware:

- Seeed WIO Terminal
- Seeed WIO Terminal Battery
- Seeed XIAO nRF52840 Sense (x2)
- Seeed Grove Air530 GPS
- Seeed Grove AHT20 Temperature
- 2x100maH LiPo (generic)
- 8GB Micro-SD Card (generic)

## Project Software Installation

### Arduino

This project uses the Arduino IDE due to lack of platform.io support for the Seeed XIAO nRF52840 Sense

When building the software, set the sketchbook location to this repository in the Arduino IDE.

This repository has the dependencies listed using Git submodules. These dependencies can be fetched with the following commands.

```bash
git submodule init
git submodule update --remote
```

In the Arduino IDE you will need to add the support for the boards used in the project. To do this go to File->Preferences and copy 
the URLs into the *Additional Board Manager URLs*.

```bash
https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
https://github.com/ambiot/ambd_arduino/raw/master/Arduino_package/package_realtek.com_amebad_index.json
```

Then in the Tools->Board->Board Manager add the following platforms

```bash
Seeed SAMD Boards
Seeed nRF52 Boards
Realtek Ameba Boards
```

The Seeed package for the XIAO comes with an outdated version of *Seeed_Arduino_FreeRTOS*. This cannot be pinned as a submodule 
because it creates an issue with the pre-installed version. This version needs to be manually updated in the Arduino installation 
directory. You will need to find this library and replace it with v2.0.0.

### Python

The Python ride-parsing software uses Poetry to track dependencies. All dependencies can be installed with Poetry except for the 
tkinter dependency for GUI creation with Matplotlilb. This will need to be installed on your system manually. I used *Ubuntu 22.04* 
for development so I installed this package with *apt*.

Poetry can be installed with pip:

```bash
pip3 install poetry
```

Then when inside of the ride-parsing software directory use Poetry: 

```bash
poetry install
poetry shell
```