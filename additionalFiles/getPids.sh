#!/bin/bash

out=$2

pids=$(ps -aux | grep roslaunch | cut -d ' ' -f $1)

echo "$pids" > $out
