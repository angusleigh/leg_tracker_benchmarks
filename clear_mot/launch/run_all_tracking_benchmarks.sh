#!/bin/bash


name=general_tracking_moving_individual_leg.launch
echo $name
roslaunch clear_mot $name  2> /dev/null | grep "Aggregated"

name=general_tracking_moving_joint_leg.launch
echo $name
roslaunch clear_mot $name  2> /dev/null | grep "Aggregated"

name=general_tracking_moving_leg_detector.launch
echo $name
roslaunch clear_mot $name  2> /dev/null | grep "Aggregated"

name=general_tracking_stationary_individual_leg.launch
echo $name
roslaunch clear_mot $name  2> /dev/null | grep "Aggregated"

name=general_tracking_stationary_joint_leg.launch
echo $name
roslaunch clear_mot $name  2> /dev/null | grep "Aggregated"

name=general_tracking_stationary_leg_detector.launch
echo $name
roslaunch clear_mot $name  2> /dev/null | grep "Aggregated"

name=person_following_indoor_individual_leg.launch
echo $name
roslaunch clear_mot $name  2> /dev/null | grep "Aggregated"

name=person_following_indoor_joint_leg.launch
echo $name
roslaunch clear_mot $name  2> /dev/null | grep "Aggregated"

name=person_following_indoor_leg_detector.launch
echo $name
roslaunch clear_mot $name  2> /dev/null | grep "Aggregated"

name=person_following_outdoor_individual_leg.launch
echo $name
roslaunch clear_mot $name  2> /dev/null | grep "Aggregated"

name=person_following_outdoor_joint_leg.launch
echo $name
roslaunch clear_mot $name  2> /dev/null | grep "Aggregated"

name=person_following_outdoor_leg_detector.launch
echo $name
roslaunch clear_mot $name  2> /dev/null | grep "Aggregated"



