#!/usr/bin/python

import rospy
import rosbag
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Int32
from tf.msg import tfMessage
from leg_tracker.msg import Person
from visualization_msgs.msg import Marker
import random
import tf
import sys, shutil 
from threading import Lock
import os


class AnnotateRosbags:    
    def __init__(self):  

        self.prev_max_marker_id = 0
        self.bag_empty = False

        self.colour = (random.random(), random.random(), random.random())
        self.cur_scan_msg_time = None
        self.next_scan_msg_time = None
        self.annotation_id = int(random.random()*999999)#None
        self.finished_successfully = False
        self.next_scan_msg = None
        self.mutex = Lock()
        self.prev_annotation_id = None
        self.found_miss = False
        self.found_fp = False
        self.found_id_switch = False
        self.prev_time_between_scans = None

        self.to_save_to_savebag = []

        rospy.init_node('annotate_rosbags', anonymous=True)

        # Give Rviz time to boot
        rospy.sleep(3) # not sure this is necessary

        # Read in ROS params
        self.readbag_filename = rospy.get_param("readbag_filename", "error") # anything we can do if it does read-in properly?
        self.savebag_filename = rospy.get_param("savebag_filename", self.readbag_filename) # by default, we assume they want to save in-place
        self.scan_topic = rospy.get_param("scan_topic", "scan")
        self.fixed_frame = rospy.get_param("fixed_frame", "laser")
        self.display_mode = rospy.get_param("display_mode", False)

        # Publisher to view messages from rosbag in Rviz
        self.laser_pub = rospy.Publisher(self.scan_topic, LaserScan)     
        self.tf_pub = rospy.Publisher("tf", tfMessage)
        self.marker_pub = rospy.Publisher("visualization_marker", Marker)

        self.listener = tf.TransformListener()        

        # Open the bag file to be annotated
        self.readbag = rosbag.Bag(self.readbag_filename) 
        self.msg_gen = self.readbag.read_messages()

        # Open the bag file to write finished annotations to
        if self.savebag_filename == self.readbag_filename:
            self.savebag = rosbag.Bag(self.savebag_filename + ".temp", 'w')
        else:
            self.savebag = rosbag.Bag(self.savebag_filename, 'w')

        # Subscriber to get annotations. 
        self.map_click_sub = rospy.Subscriber("clicked_point", PointStamped , self.clicked_point_callback)
        self.annnotation_control_sub = rospy.Subscriber("annotation_control", String , self.annnotation_control_callback)
        self.annnotation_id_sub = rospy.Subscriber("annotation_id", Int32 , self.annnotation_id_callback)

        # To make sure bagfiles are closed properly
        rospy.on_shutdown(self.close_bags)

        # So node doesn't shut down
        rospy.spin()     


    def __del__(self):
        self.close_bags()


    def close_bags(self, not_using_this_var=None):
        self.readbag.close()
        self.savebag.close()

        if self.finished_successfully and not self.display_mode:
            rospy.loginfo("Finished successfully")
            if self.savebag_filename == self.readbag_filename:
                os.remove(self.readbag_filename)
                os.rename(self.savebag_filename + ".temp", self.savebag_filename)
        else:
            rospy.logwarn("Did NOT finish succesfully! No changes will be saved!")            
            if self.savebag_filename == self.readbag_filename:
                os.remove(self.savebag_filename + ".temp")
            else:
                os.remove(self.savebag_filename)


    def annnotation_control_callback(self, msg):
        if self.next_scan_msg_time is None or self.cur_scan_msg_time is None:
            self.nextScanMsg()
            return

        # Reset some variables
        self.found_miss = False
        self.found_id_switch = False
        self.found_fp = False

        if msg.data == "Next":
            self.clear_old_markers()  # Clear published markers before we skip ahead
            self.nextScanMsg()
        elif msg.data == "Forwards":
            self.clear_old_markers() # Clear published markers before we skip ahead
            self.nextScanMsg(10)
        elif msg.data == "End":
            self.clear_old_markers() # Clear published markers before we skip ahead
            self.nextScanMsg(sys.maxint)
        elif msg.data == "Clear\nprevious":
            if self.to_save_to_savebag:
                rospy.loginfo("Clearing previous annotation!")
                # Clear any previous markers which were still breing published when the previous click was registered
                clear_time = self.to_save_to_savebag[0][2]
                del self.to_save_to_savebag[:]
                self.clear_old_markers(time=clear_time)
            else:
                rospy.loginfo("No previous annotation exists! Nothing cleared")
        elif msg.data == "Next\nmiss":
            if self.display_mode:
                while self.found_miss == False and not self.bag_empty:
                    self.nextScanMsg()
            else:
                rospy.loginfo("option not available unless in display mode viewing results from a clear_mot evaluation")
        elif msg.data == "Next\nid_switch":  
            if self.display_mode:
                while self.found_id_switch == False and not self.bag_empty:
                    self.nextScanMsg()
            else:
                rospy.loginfo("option not available unless in display mode viewing results from a clear_mot evaluation")
        elif msg.data == "Next\nfp":  
            if self.display_mode:
                while self.found_fp == False and not self.bag_empty:
                    self.nextScanMsg()
            else:
                rospy.loginfo("option not available unless in display mode viewing results from a clear_mot evaluation")

            

    def annnotation_id_callback(self, msg):
        self.annotation_id = msg.data
        rospy.loginfo("annotation id: " + str(self.annotation_id))

        # Set colour based on annotation_id:
        # Want the same colours for the same annotation_ids even when running the program multiple times
        random.seed(self.annotation_id)
        self.colour = (random.random(), random.random(), random.random())

        self.clear_old_markers()


    def clicked_point_callback(self, clicked_point_msg):
        if self.next_scan_msg_time is None or self.cur_scan_msg_time is None:
            self.nextScanMsg()
            return

        # Save messages previously set to write to rosbag, as long as they haven't been cleared elsewhere already
        for topic, msg, time in self.to_save_to_savebag:
            self.savebag.write(topic, msg, time)
        del self.to_save_to_savebag[:]

        # Calc time to save messages to rosbags:
        time = (self.next_scan_msg_time - self.cur_scan_msg_time)/2 + self.cur_scan_msg_time

        # Save person's position in savebag
        person = Person()
        person.id = self.annotation_id
        person.pose.position = clicked_point_msg.point
        self.to_save_to_savebag.append(("/ground_truth_people_tracks", person, time))

        # Rviz marker to save to the rosbag            
        m2 = Marker()
        m2.header.frame_id = self.fixed_frame
        m2.id = self.annotation_id
        m2.ns = "ground_truth_people_tracks_markers"
        m2.type = Marker.SPHERE
        if self.bag_empty:   # In this case we'll be exiting right after and won't have time to clear the marker   
            m2.lifetime = rospy.Duration(0.25); 
        m2.scale.x = 0.2
        m2.scale.y = 0.2
        m2.scale.z = 0.3
        m2.color.r = self.colour[0]
        m2.color.g = self.colour[1]
        m2.color.b = self.colour[2]
        m2.color.a = 1.0
        m2.pose.position.x = person.pose.position.x 
        m2.pose.position.y = person.pose.position.y 
        m2.pose.position.z = 0.1                
        self.to_save_to_savebag.append(("/visualization_marker", m2, time))

        # Rviz marker showing id number
        m3 = Marker()
        m3.header.frame_id = self.fixed_frame
        m3.id = self.annotation_id
        m3.ns = "annotation_ids"
        m3.type = Marker.TEXT_VIEW_FACING
        if self.bag_empty:  # In this case we'll be exiting right after and won't have time to clear the marker
            m3.lifetime = rospy.Duration(0.25);
        m3.scale.x = 0.3
        m3.scale.y = 0.3
        m3.scale.z = 0.3
        m3.color.r = 1.0
        m3.color.g = 1.0
        m3.color.b = 1.0
        m3.color.a = 1.0
        m3.pose.position.x = person.pose.position.x 
        m3.pose.position.y = person.pose.position.y 
        m3.pose.position.z = 0.3                
        m3.text = str(self.annotation_id)
        self.to_save_to_savebag.append(("/visualization_marker", m3, time))

        # Rviz marker of where annotator clicked to display to rviz temporarily (not saving to rosbag)
        m = Marker()
        m.header.frame_id = self.fixed_frame
        m.id = self.annotation_id
        m.ns = "clicked_points"
        m.type = Marker.SPHERE
        m.lifetime = rospy.Duration(0.05); # Just a short duration to show in rviz
        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.3
        m.color.r = self.colour[0]
        m.color.g = self.colour[1]
        m.color.b = self.colour[2]
        m.color.a = 1.0
        m.pose.position.x = person.pose.position.x 
        m.pose.position.y = person.pose.position.y 
        m.pose.position.z = 0.1
        self.marker_pub.publish(m)

        self.prev_annotation_id = self.annotation_id # So we know which markers need to be removed on next iteration

        self.nextScanMsg()

    def clear_old_markers(self, time=None):
        # Save messages previously set to write to rosbag, as long as they haven't been cleared elsewhere already
        for topic, msg, time_2 in self.to_save_to_savebag:
            self.savebag.write(topic, msg, time_2)
        del self.to_save_to_savebag[:]

        if self.prev_annotation_id is not None:           
            if time is None:
                time = (self.next_scan_msg_time - self.cur_scan_msg_time)/2 + self.cur_scan_msg_time
            m = Marker()
            m.header.frame_id = self.fixed_frame    
            m.id = self.prev_annotation_id            
            m.ns = "ground_truth_people_tracks_markers"
            m.action = m.DELETE      
            self.savebag.write("/visualization_marker", m, time)
            m.ns = "annotation_ids"
            self.savebag.write("/visualization_marker", m, time)
            self.prev_annotation_id = None



    # Iterate through the rosbag until we hit a scan message
    # Save that message and break. Play it first time nextScanMsg() is called again.
    # We don't publish and save the scan message found so all the other visualizations markers corresponding to the previous scan 
    # can be displayed and the annotater can annotate based on them.
    # Publish all other messages to Rviz along the way (if display=True)
    def nextScanMsg(self, num_scan_msgs=1, display=True):
        self.mutex.acquire() # TODO necessary?

        if self.bag_empty:         
            # Finish writing any messages set to be saved
            # Don't worry about clearing any markers written at the end because all final markers publish should have a limited lifespan
            for topic, msg, time in self.to_save_to_savebag:
                self.savebag.write(topic, msg, time)

            # Shut down and close rosbags safetly
            self.finished_successfully = True                    
            rospy.loginfo("End of readbag file, shutting down")
            rospy.signal_shutdown("End of readbag file, shutting down")

        # We need <self.prev_time_between_scans> for if we hit the end of the bag and need to extrapolate time
        if self.next_scan_msg_time and self.cur_scan_msg_time:
            self.prev_time_between_scans = self.next_scan_msg_time - self.cur_scan_msg_time 

        # First display and save the next_scan_msg
        if self.next_scan_msg:
            topic, msg, time = self.next_scan_msg
            self.cur_scan_msg_time = time                   
            self.next_scan_msg = None
            self.savebag.write(topic, msg, time)
            self.laser_pub.publish(msg)

        # Iterate through rosbag until we've found num_scan_msgs scan messages
        for msg_num in xrange(0, num_scan_msgs):
            if self.bag_empty:
                break

            marker_id = 0
            topic = None
            while topic !=  self.scan_topic and not self.bag_empty:
                try:
                    # Get the next message from the rosbag
                    topic, msg, time = next(self.msg_gen)

                    if topic ==  self.scan_topic:
                        if msg_num < num_scan_msgs-1: # We still have some messages to go 
                            # Display and save scan message to rosbag
                            self.savebag.write(topic, msg, time)
                            self.cur_scan_msg_time = time                   
                            if display:
                                self.laser_pub.publish(msg)
                        else:
                            # Last iteration. Save scan message for when nextScanMsg() is called again
                            self.next_scan_msg = (topic, msg, time)
                            self.next_scan_msg_time = time
                    else:
                        # Save message to savebag 
                        self.savebag.write(topic, msg, time)

                        # Publish it for display in Rviz
                        if display:
                            if topic == "/tf":
                                self.tf_pub.publish(msg)
                            elif topic == "/visualization_marker":
                                self.marker_pub.publish(msg)     

                                if msg.ns == "clear_mot" and msg.type == Marker.TEXT_VIEW_FACING:
                                    if "miss" in msg.text:
                                        self.found_miss = True
                                    elif "fp" in msg.text:
                                        self.found_fp = True
                                    elif "id" in msg.text:
                                        self.found_id_switch = True

                except StopIteration:
                    # Passed the last message in the rosbag
                    # Don't close everything yet, but give the annotater one more chance to make an annotation before closing down
                    self.bag_empty = True
                    # Extrapolate time into future
                    if self.prev_time_between_scans is not None:
                        self.next_scan_msg_time = self.prev_time_between_scans + self.cur_scan_msg_time
                    else:
                        rospy.logerr("Error! It appears we didn't read in > 1 scan message. Is scan_topic set correctly?")

            # Clear previously published people markers
            if self.prev_max_marker_id > marker_id:
                for m_id in xrange(marker_id, self.prev_max_marker_id):
                    marker = Marker()
                    marker.header.frame_id = self.fixed_frame
                    marker.ns = "clear_mot_display"
                    marker.id = m_id
                    marker.action = marker.DELETE   
                    self.marker_pub.publish(marker)     
            self.prev_max_marker_id = marker_id

        self.mutex.release()





if __name__ == '__main__':
    ar = AnnotateRosbags()




