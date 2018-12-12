# MKS-Robin-Bootloader

This repository contains tool that allow you to create a backup copy of bootloader from your MKS Robin board (STM32F103ZET6).
Backup copy of bootloader can be used to return original board's functionality after you had programm board with an alternative firmware.

# HowTo
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
