#!/bin/bash
if [ -e "/dev/ttyACM0" ]
then
  echo "CDC ACM is Loaded";
  rmmod cdc_acm;
  modprobe -r cdc-acm;
  modprobe -r xr_usb_serial_common;
  modprobe xr_usb_serial_common;
else
  echo "CDC ACM Already Unloaded";
fi

