#!/bin/bash

if which leapd > /dev/null; then
    echo "the Leapmotion driver already exists."
else
    echo "installing the Leapmotion driver"
    if $(uname -m | grep '64'); then
        sudo dpkg --install ./leapsetup/Leap-0.8.0-x64.deb
    else
        sudo dpkg --install ./leapsetup/Leap-0.8.0-x86.deb
    fi
fi

rosmake rosleap_msg && rosmake rosleapm_src && rosmake rosleap_samples