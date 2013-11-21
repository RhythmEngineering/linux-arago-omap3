#!/bin/bash

# setup_build.sh (v1.0)
# Utility script for kernel builds

export PATH=$PATH:/usr/local/ti-sdk-am3517-evm/linux-devkit/bin/
export CROSS_COMPILE=arm-arago-linux-gnueabi-
export ARCH=arm
export LOADADDR=80008000
