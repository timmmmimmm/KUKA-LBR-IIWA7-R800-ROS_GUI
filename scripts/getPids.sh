#!/bin/bash

out=~/iiwa_stack_ws/src/iiwa_gui/scripts/pids.txt

pids=$(ps -aux | grep roslaunch | cut -d ' ' -f $1)

echo "$pids" > $out
echo $pids
