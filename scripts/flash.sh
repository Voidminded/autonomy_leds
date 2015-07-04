#!/bin/bash
sudo dfu-programmer atmega32u4 erase && sudo dfu-programmer atmega32u4 flash leds_firmware.hex
