#!/bin/bash

echo ""
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PWD" >> ~/.bashrc
echo "copying..."
sleep 1
source ~/\.bashrc
