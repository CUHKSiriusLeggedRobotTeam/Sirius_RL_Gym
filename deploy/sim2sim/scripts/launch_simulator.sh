#!/bin/bash

if pgrep -x "iox-roudi" > /dev/null; then
    echo "Found existing iox-roudi. Killing it..."
    sudo pkill iox-roudi
    sleep 1
fi

sudo bash -c '
cd ../simulator/bin
iox-roudi &
sleep 1
LD_LIBRARY_PATH=../lib ./sim_test &
LD_LIBRARY_PATH=../lib ./sim_ctrl
'