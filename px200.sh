#!/bin/bash

KERNEL=$(uname -r)
echo Kernel Version $KERNEL

echo Loading PX200 Driver
sudo insmod ~/px200/src/px200.ko

