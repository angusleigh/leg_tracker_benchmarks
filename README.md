# Leg Tracker Benchmarks
for ROS Melodic

- Repo for recreating results in paper: 
A. Leigh, J. Pineau, N. Olmedo and H. Zhang, Person Tracking and Following with 2D Laser Scanners, International Conference on Robotics and Automation (ICRA), Seattle, Washington, USA, 2015

Usage
--------------------
### Installation
- `cd ~/catkin_ws/src`
- `git clone --recurse-submodules git@github.com:angusleigh/leg_tracker_benchmarks.git`
- `unzip "leg_tracker_benchmarks/benchmark_rosbags/annotated/*.zip" -d leg_tracker_benchmarks/benchmark_rosbags/annotated/`
- install dependencies
    - `sudo apt install python-scipy`
    - `sudo pip install pykalman`
    - `sudo apt install ros-melodic-bfl`
    - `sudo apt install qt4-default`
    - `sudo apt install ros-melodic-kalman-filter`
    - `sudo apt install ros-melodic-easy-markers`
- `cd ~/catkin_ws`
- `catkin_make`

If you currently have the leg_tracker repo in your catkin_workspace, you will have to move it because leg_tracker_benchmarks has a version of the same package in it as well.

### Running the tracking benchmarks
- `cd benchmark_launch`
- `roslaunch tracking_benchmarks/general_tracking_moving_1_joint_leg.launch`
- or launch one of the other configurations in the `tracking_benchmarks` and `runtime_benchmarks` directories

This will read-in the corresponding rosbag in benchmark_rosbags/annotated, publish it in a deterministic manner (so there's no race conditions or time-dependant inconsistencies between runs) using playback_and_record_tracked.py. The tracker's position estimates will be saved to benchmark_rosbags/annotated_and_tracked.

Note that there's a duplicate set of launch files for runtime benchmarks, which just run everything as fast as possible without worrying about race conditions. 

Feel free to uncomment the command to view output in Rviz in the launch file to see the visualizations.

CLEAR MOT results should be very close to the reference paper. To reproduce results exactly, checkout the oldest commit in this repo. Caveat is that it is not as well documented and the code is messier than at head.

### Running the benchmark CLEAR MOT evaluation
- `roslaunch clear_mot [benchmark name]`

This will read-in the tracker output data from benchmarks/annotated_and_tracked, evaluate the CLEAR MOT metric performance and write the output to benchmarks/annotated_and_tracked_and_clear_mot. It will also print the CLEAR MOT scores to the terminal.

You can visualize the CLEAR MOT errors and results using 

- `roslaunch clear_mot view_results.launch`

This launches an Rviz window with a panel which allows you to step through the data and shows all the CLEAR MOT data associations and errors. To view different files, change the "readbag_filename" param in launch file.


### Annotating ground truth in new data
- `roslaunch annotate_rosbags annotate.launch`

This is the tool I used to annotate the ground truth tracks in the rosbags. After launching, you can simply click on the person's location in the map and it will save the location of your click, and advance to the next scan.

I also used split.launch and merge.launch for rosbags which were too big and unwieldy. This way, you can split long rosbags, annotate them individually and merge them afterwards. Then, if you make a mistake in your annotation, it's much easier fix because it's isolated to one rosbag.


### Camera data

Camera data is also available from two separate webcams mounted immediately on top of the laser scanner. Only the compressed image stream is included to keep the files to a reasonable size. One way to get the raw image topics from the compressed topics is using image_transport

- `rosrun image_transport republish compressed in:=/vision/image _image_transport:=compressed raw out:=/vision/image`

You can also view them with the image_view package 

- `rosrun image_view image_view image:=/usb_cam_node2/image_raw compressed`

Unfortunately, the cameras are not calibrated as we only anticipated using them as a rough guide for ground-truth annotation and not for image processing. 

To keep this repo to a reasonable size and so folks who don't want the images aren't forced to download them, the images are stored in a separate repo on BitBucket: https://bitbucket.org/aleigh/leg_tracker_benchmarks_jpg. It's about 1Gb in size. 

To use the rosbags with the camera data, you should copy them into the benchmark_rosbags/annotated folder and replace the existing rosbags there.
