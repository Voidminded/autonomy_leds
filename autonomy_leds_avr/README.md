## Introduction

This package includes the [rosserial](http://wiki.ros.org/rosserial) based firmware for _autonomy_leds_ LED strip driver board. The driver board is based on ATMega32u4 AVR micro-controller. The firmware is developed using [rosserial_client](http://wiki.ros.org/rosserial_client) library for AVR micro-controllers and is fully integrated into catkin build system.

## Board Information

LED Strip: http://www.adafruit.com/products/2239

Driver board: TBA

## udev rules

Please install the udev rule file located in `autonomy_leds_tools/udev` to your system based on instructions provided by the accompanying README file. These rules give any user in `dialout` group on the host machine access to the driver board over USB, both for programming it and interfacing with it.

## Firmware Build Instructions

Pre-requirements:

- ROS Indigo (or newer)
- [catkin_tools](https://catkin-tools.readthedocs.org/en/latest/) `sudo apt-get install python-catkin-tools`
- gcc-avr toolchain (will be automatically installed by rosdep)

```bash
$ mkdir -p ~/autonomy_leds_ws/src
$ cd ~/autonomy_leds_ws
$ catkin init
$ cd src
$ git clone https://github.com/AutonomyLab/autonomy_leds
$ cd ..
$ rosdep update && rosdep install --from-paths src -i
$ catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

The above procedure will install all required system-wide dependencies using `rosdep`. There is no need to do the `rosdep` step on recompiles. The hex file for the firmware (`leds_firmware.hex`) will be placed in `autonomy_leds_ws/devel/share/autonomy_leds_avr/firmware` after successful compilation.

## Flashing Instructions

Pre-requirement: dfu-programmer

- Reboot the board into DFU bootloader mode by reseting the board while JP1 is short circuited.

- Check if the device is in DFU mode

```
$ lsusb | grep -i atmel
```

- Erase the flash

```
$ dfu-programmer atmega32u4 erase
```

- Flash the hex file

```
$ dfu-programmer atmega32u4 flash leds_firmware.hex
```

(Alternate way)

```
$ rosrun autonomy_leds_tools flash.sh
```

## Misc

### Checking the flash/data usage

```
$ avr-size -C --mcu=atmega32u4 leds_firmware.elf
```

## Credits

- [LUFA Library](http://www.fourwalledcubicle.com/LUFA.php) - MIT License - Adapted from [here](https://github.com/mrjogo/rosserial_avr_tutorial/tree/lufa)
- [light_apa102_AVR](https://github.com/cpldcpu/light_ws2812/tree/master/light_apa102_AVR) - GPLv2 License