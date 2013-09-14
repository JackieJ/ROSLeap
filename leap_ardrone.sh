#!/bin/bash

echo "downloading ardrone_autonomy package......"
git clone git://github.com/tum-vision/ardrone_autonomy.git
echo "building ardrone_autonomy package.......ARSDroneLib and collatorals"
cd ardrone_autonomy && ./build_sdk.sh && cd ..
rosmake ardrone_autonomy
echo "downloading the tum package......"
git clone git://github.com/tum-vision/tum_ardrone.git
echo "rosmaking the tum package......."
sudo apt-get install liblapack-dev && rosmake tum_ardrone