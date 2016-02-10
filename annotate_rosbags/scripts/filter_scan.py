#!/usr/bin/python

import rospy
import rosbag


class FilterScan:    
    def __init__(self):  
        part = 1
        part_start_time = None

        rospy.init_node('filter_scan', anonymous=True)

        readbag_filename = "/home/angus/rosbags/webcam_tests_june_17/annotation/benchmark_3.bag"

        readbag = rosbag.Bag(readbag_filename) 
        savebag_filename = readbag_filename.replace('.bag', '_filtered_scan.bag')
        savebag = rosbag.Bag(savebag_filename, 'w')

        for topic, msg, t in readbag.read_messages():
            if topic == "/ground_truth_people_tracked":
                new_ranges = []
                for idx, dist in enumerate(msg.ranges):
                    if idx > 35:
                        new_ranges.append(dist)
                    else:
                        new_ranges.append(0)
                msg.ranges = tuple(new_ranges)
                   
            savebag.write(topic, msg, t)

        readbag.close()
        savebag.close()

        rospy.loginfo("Finished filtering scan")


if __name__ == '__main__':
    fs = FilterScan()




