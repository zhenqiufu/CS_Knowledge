#!/bin/bash

roslaunch dp_core record.launch &
sleep 2

echo "bhand controller starting success!"

roslaunch dp_core my.launch &
sleep 0.1
wait
exit 0
