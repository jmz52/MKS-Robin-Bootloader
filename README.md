# MKS-Robin-Bootloader

This repository contains tool that allows you to create a backup copy of bootloader from your MKS Robin board (STM32F103ZET6).
Backup copy of bootloader can be used to return original board's functionality after you had program board with an alternative firmware.

## Source code
The 'Robin' directory contains PlatformIO project with with custom scripts that are required to make MKS Robin-compatible encrypted firmware.
During final stage of build process the 'firmware.bin' file is encrypted and saved as 'Robin.bin' that can be used with built-in bootloader.

## Usage
 - copy Robin.bin to SD card
 - insert SD card to MKS Robin board
 - reboot MKS Robin board
 - wait until firmware upgrade is completed
 - wait until on-board LED starts blinking
 - remove SD card and copy bootload.bin to you computer for safekeeping

 - copy Robin.bin from latest firmware to SD card
 - insert SD card to MKS Robin board
 - reboot MKS Robin board
 - wait until firmware upgrade is completed
