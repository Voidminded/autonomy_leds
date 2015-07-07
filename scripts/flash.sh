#!/bin/bash
sudo dfu-programmer atmega32u4 erase && sudo dfu-programmer atmega32u4 flash ../firmware/leds_firmware.hex
