#!/usr/bin/python

import rospy
import rosbag
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Int32
from tf.msg import tfMessage
from leg_tracker.msg import Person, PersonArray
from visualization_msgs.msg import Marker
import random
import tf
import sys, shutil 
from threading import Lock
import os
import timeit



class PlayBackAndRecordTracked:    
    def __init__(self):  

        self.prev_marker_id = 0
        self.prev_person_marker_id = 0
        self.bag_empty = False
        self.person_colours = {}
        random.seed(1) # So we have the same psuedorandom numbers each time

        self.cur_scan_msg_time = None
        self.next_scan_msg_time = None
        self.finished_successfully = False
        self.next_scan_msg = None
        self.mutex = Lock()

        self.tracking_msg_count = 0
        self.tracking_time_cum = 0
        self.worst_tracking_time = 0
        self.tracking_started = False

        self.to_save_to_savebag = []

        rospy.init_node('record_tracked', anonymous=True)

        # Give Rviz time to boot
        # rospy.sleep(3) # not sure this is necessary

        # Read in ROS params
        self.readbag_filename = rospy.get_param("readbag_filename", None) 
        self.savebag_filename = rospy.get_param("savebag_filename", None) 
        self.scan_topic = rospy.get_param("scan_topic", None)
        self.record_logs = rospy.get_param("record_logs", False)

        if self.readbag_filename is None or self.scan_topic is None:
            rospy.logerr("readbag_filename, savebag_filename or scan_topic not provided in parameters")

        # Publisher to view messages from rosbag in Rviz
        self.laser_pub = rospy.Publisher(self.scan_topic, LaserScan)     
        self.tf_pub = rospy.Publisher("tf", tfMessage)
        self.marker_pub = rospy.Publisher("visualization_marker", Marker)

        # Open the bag file to be annotated
        self.readbag = rosbag.Bag(self.readbag_filename) 
        self.msg_gen = self.readbag.read_messages()

        # Open the bag file to write tracked stuff to
        if self.savebag_filename is not None:
            self.savebag = rosbag.Bag(self.savebag_filename, 'w')

        # To make sure bagfiles are closed properly
        rospy.on_shutdown(self.close_bags)

        # Subscribers (note: the location this is declared is important)
        self.people_tracked_sub = rospy.Subscriber('people_tracked', PersonArray, self.people_tracked_callback)    

        rospy.sleep(2.0)  # To make sure the tracking node has time to boot up

        # Publish the scan message which should start a chain reaction of callbacks
        self.nextScanMsg()
        self.tic = timeit.default_timer()
        self.nextScanMsg() # Should be copied twice: not a mistake. The very first calling of nextScanMsg will not publish anything

        # So node doesn't shut down and to keep callbacks active
        rospy.spin()     


    def __del__(self):
        self.close_bags()


    def close_bags(self, not_using_this_var=None):
        self.readbag.close()
        if self.savebag_filename is not None:
            self.savebag.close()         

        if self.tracking_msg_count != 0:
            rospy.loginfo("avg tracking time: %.2fHz worst: %.2fHz based on %d msgs", float(self.tracking_msg_count)/self.tracking_time_cum, 1/self.worst_tracking_time, self.tracking_msg_count)


    def people_tracked_callback(self, people_tracked_msg):    
        toc = timeit.default_timer()

        # Don't start keeping tracking of tracking time until at least one person has started to be tracked
        # This is because the tracking programs are often slow to startup when no tfs have been published yet
        if len(people_tracked_msg.people) > 0:
            self.tracking_started = True

        if self.tracking_started:
            tracking_time = toc - self.tic
            self.tracking_time_cum += tracking_time
            self.tracking_msg_count += 1
            if tracking_time > self.worst_tracking_time:
                self.worst_tracking_time = tracking_time
                rospy.loginfo("***************************Worst tracking time so far: %.2f***************************", self.worst_tracking_time)

        if self.savebag_filename is not None:
            save_time = (self.next_scan_msg_time - self.cur_scan_msg_time)/2. + self.cur_scan_msg_time
            people_tracked_msg.header.stamp = save_time
            self.savebag.write('/people_tracked', people_tracked_msg, save_time)

            # save rviz markers
            marker_id = 0     
            for person in people_tracked_msg.people:
                self.tracking_started = True

                if person.id not in self.person_colours:
                    self.person_colours[person.id] = (random.random(), random.random(), random.random())
                marker = Marker()
                marker.header.frame_id = people_tracked_msg.header.frame_id
                marker.header.stamp = save_time
                marker.ns = "playback_and_record_tracked"
                marker.color.r = self.person_colours[person.id][0]
                marker.color.g = self.person_colours[person.id][1]
                marker.color.b = self.person_colours[person.id][2]                                    
                marker.color.a = 1
                marker.pose.position.x = person.pose.position.x
                marker.pose.position.y = person.pose.position.y
                for i in xrange(2): # publish two markers per person: one for body and one for head
                    marker.id = marker_id 
                    marker_id += 1
                    if i==0: # cylinder for body shape
                        marker.type = Marker.CYLINDER
                        marker.scale.x = 0.15
                        marker.scale.y = 0.15
                        marker.scale.z = 0.6
                        marker.pose.position.z = 0.6
                    else: # sphere for head shape
                        marker.type = Marker.SPHERE
                        marker.scale.x = 0.15
                        marker.scale.y = 0.15
                        marker.scale.z = 0.15                
                        marker.pose.position.z = 0.975
                    self.savebag.write('/visualization_marker', marker, save_time)

                # Text showing person's ID number 
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.TEXT_VIEW_FACING
                marker.text = str(person.id)
                marker.scale.z = 0.2         
                marker.pose.position.z = 1.3
                self.savebag.write('/visualization_marker', marker, save_time)

            # Clear previously published people markers
            for m_id in xrange(marker_id, self.prev_person_marker_id):
                marker = Marker()
                marker.header.stamp = people_tracked_msg.header.stamp                
                marker.header.frame_id = people_tracked_msg.header.frame_id
                marker.ns = "playback_and_record_tracked"
                marker.id = m_id
                marker.action = marker.DELETE   
                self.savebag.write('/visualization_marker', marker, save_time)

            self.prev_person_marker_id = marker_id   

        self.nextScanMsg()

        

    # Iterate through the rosbag until we hit a scan message
    # Save that message and break. Play it first time nextScanMsg() is called again.
    # Publish all other messages to Rviz along the way 
    def nextScanMsg(self):
        self.mutex.acquire() # TODO necessary?

        # First display and save <next_scan_msg>
        # It is now referred to as the cur_scan_msg 
        # And we'll iterate through the rosbag until we reach the next_scan_msg
        if self.next_scan_msg is not None:
            topic, msg, time = self.next_scan_msg
            self.cur_scan_msg_time = time                   
            self.next_scan_msg = None
            if self.savebag_filename is not None:
                self.savebag.write(topic, msg, time)

            self.tic = timeit.default_timer()

            self.laser_pub.publish(msg)

        if self.bag_empty:         
            # Shut down and close rosbags safetly
            rospy.loginfo("End of readbag file, shutting down")
            rospy.signal_shutdown("End of readbag file, shutting down")


        marker_id = 0
        topic = None
        while topic !=  self.scan_topic and not self.bag_empty:
            try:
                # Get the next message from the rosbag
                topic, msg, time = next(self.msg_gen)

                if topic ==  self.scan_topic:
                    self.next_scan_msg = (topic, msg, time)
                    self.next_scan_msg_time = time     
                else:
                    # Save message to savebag 
                    if self.savebag_filename is not None:
                        self.savebag.write(topic, msg, time)

                    # Publish them
                    if topic == "/tf":
                        self.tf_pub.publish(msg)         
                    elif topic == "/visualization_marker":
                        marker = Marker()
                        marker.header = msg.header
                        marker.ns = msg.ns
                        marker.id = msg.id
                        marker.type = msg.type          
                        marker.action = msg.action
                        marker.pose = msg.pose
                        marker.scale = msg.scale
                        marker.color = msg.color
                        marker.lifetime = msg.lifetime
                        marker.frame_locked = msg.frame_locked
                        marker.points = msg.points
                        marker.colors = msg.colors
                        marker.text = msg.text
                        marker.mesh_resource = msg.mesh_resource
                        marker.mesh_use_embedded_materials = msg.mesh_use_embedded_materials
                        self.marker_pub.publish(marker)


            except StopIteration:
                # Passed the last message in the rosbag
                self.bag_empty = True             

        # Clear previously published people markers
        if self.prev_marker_id > marker_id:
            for m_id in xrange(marker_id, self.prev_marker_id):
                marker = Marker()
                marker.header.frame_id = self.fixed_frame
                marker.ns = "clear_mot_display"
                marker.id = m_id
                marker.action = marker.DELETE   
                self.marker_pub.publish(marker)     
        self.prev_marker_id = marker_id

        self.mutex.release()





if __name__ == '__main__':
    pbart = PlayBackAndRecordTracked()




