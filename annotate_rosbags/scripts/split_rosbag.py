#!/usr/bin/python

import rospy
import rosbag


class SplitRosbags:    
    def __init__(self):  
        part = 1
        part_start_time = None

        rospy.init_node('split_rosbags', anonymous=True)

        readbag_filename = rospy.get_param("readbag_filename", "")
        split_interval = rospy.get_param("split_interval", 20.0)

        readbag = rosbag.Bag(readbag_filename) 
        savebag_filename = readbag_filename.replace('.bag', '_part-' + str(part) + '.bag')
        savebag = rosbag.Bag(savebag_filename, 'w')

        for topic, msg, t in readbag.read_messages():
            savebag.write(topic, msg, t)

            if part_start_time == None:
                part_start_time = t.to_sec()

            if  t.to_sec() > part_start_time + split_interval:
                # Move onto the next part in a new savebag
                
                rospy.loginfo("Finished part: " + str(part))
                part_start_time += split_interval
                part += 1
                savebag.close()
                savebag_filename = readbag_filename.replace('.bag', '_part-' + str(part) + '.bag')
                savebag = rosbag.Bag(savebag_filename, 'w')

        readbag.close()
        savebag.close()

        rospy.loginfo("Finished splitting into " + str(part) +" parts!")


if __name__ == '__main__':
    sr = SplitRosbags()




