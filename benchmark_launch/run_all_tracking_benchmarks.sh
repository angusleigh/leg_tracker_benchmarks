#!/bin/bash
for file in tracking_benchmarks/*.launch
do
    name=${file##*/} 
    echo $name
    roslaunch leg_tracker $name
done
