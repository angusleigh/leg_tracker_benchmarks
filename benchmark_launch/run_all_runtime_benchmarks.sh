#!/bin/bash
for file in runtime_benchmarks/*.launch
do
    name=${file##*/} 
    echo $name
    roslaunch leg_tracker $name  2> /dev/null | grep "avg"
done