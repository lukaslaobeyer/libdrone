#!/bin/sh
arm-linux-gnueabi-gcc -mtune=cortex-a9 -march=armv7-a ./src/navdataserver.c
