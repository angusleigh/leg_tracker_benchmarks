#!/usr/bin/python

import rospy
import rosbag

# Merge the split rosbags and remove images
class MergeRosbags:    
    def __init__(self):  
        part = 1

        rospy.init_node('merge_rosbags', anonymous=True)

        root_rosbag = rospy.get_param("root_rosbag", "")

        # Rosbag to merge and save messages to 
        savebag_filename = root_rosbag.replace('.bag', '_annotated.bag')
        savebag = rosbag.Bag(savebag_filename, 'w')

        while True:
            try:
                readbag_filename = root_rosbag.replace('.bag', '_part-' + str(part) + '.bag')
                readbag = rosbag.Bag(readbag_filename) 

                rospy.loginfo("Merging part " + str(part))

                # Save this part to the merged rosbag
                for topic, msg, t in readbag.read_messages():
                    # Save all messages except the images
                    if topic != "/usb_cam_node2/image_raw" and topic != "/usb_cam_node3/image_raw":
                        savebag.write(topic, msg, t)                

                part += 1
                readbag.close()
            except:
                # Hopefully we finished the last part. We can close everything and exit
                break

        readbag.close()
        savebag.close()

        rospy.loginfo("Finished merging " + str(part-1) + " parts sucessfully!")


if __name__ == '__main__':
    mr = MergeRosbags()




