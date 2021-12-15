#!/bin/sh

exec /usr/bin/smartfan2 -thermal-zone $(grep k10temp /sys/class/hwmon/hwmon*/name -l)
