#!/usr/bin/python

import rospy
import rosbag
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
import tf
from tf.msg import tfMessage
from visualization_msgs.msg import Marker
from leg_tracker.msg import Person, PersonArray
from munkres import Munkres # Third party library. For the minimum matching assignment problem. To install: https://pypi.python.org/pypi/munkres 
import math
import numpy as np


class ClearMot:    
    max_match_threshold = 0.75

    def __init__(self, filenames): 
        rospy.init_node('clear_mot', anonymous=True)

        # Read in ROS params      
        self.scan_topic = rospy.get_param("scan_topic", "scan")
        self.laser_frame = None # To be read-in from the scan-header of the first scan received
        self.calc_ground_truth_marker_dist_statistics = rospy.get_param("calc_ground_truth_marker_dist_statistics", True)
        self.constrain_to_specific_angle = rospy.get_param("constrain_to_specific_angle", False)
        self.min_angle = rospy.get_param("min_angle", -180)
        self.max_angle = rospy.get_param("max_angle", 180) 
        self.min_angle = math.pi*self.min_angle/180.
        self.max_angle = math.pi*self.max_angle/180.

        # Should be persistant across multiple files if we want to calc dist to followee
        # Note that this only works when there is only one annotated person!
        if self.calc_ground_truth_marker_dist_statistics:
            self.ground_truth_msg_count = None
            self.ground_truth_cum_dist = None
            self.ground_truth_M_2n = None # from: http://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#On-line_algorithm
            self.ground_truth_avg_prev = None

        aggregated_results = []
        for readbag_filename, savebag_filename in filenames: 
            self.latest_time = None
            self.prev_marker_id = 0

            # Needed to calculate transforms
            self.tf_pub = rospy.Publisher("tf", tfMessage, queue_size=100)

            # Open the bag file to be annotated
            self.readbag = rosbag.Bag(readbag_filename)
            self.msg_gen = self.readbag.read_messages()

            # Open the bag file to write clear_mot animations to
            self.savebag = rosbag.Bag(savebag_filename, 'w')

            # To make sure bagfiles are closed properly
            rospy.on_shutdown(self.close_bags)

            correct_assignments, id_switches, misses, false_positives, MOTA, MOTP, total_objects, MOTP_dist_cum, MOTP_dist_count = self.calc_clear_mot()

            rospy.loginfo("CLEAR MOT statistics for " + readbag_filename + " :")
            rospy.loginfo("correct:%d id_switches:%d, misses:%d, FPs:%d, MOTA:%.2f%% MOTP:%.2fm total_objects:%d", correct_assignments, id_switches, misses, false_positives, 100*MOTA, MOTP, total_objects)
            aggregated_results.append((correct_assignments, id_switches, misses, false_positives, MOTA, MOTP, total_objects, MOTP_dist_cum, MOTP_dist_count))

            # Error checking
            if total_objects != (correct_assignments + id_switches + misses):
                rospy.logerr("Problem! Maybe a bug in clear_mot.py? correct + id_switches + misses != total_objects  when it should be")

        # Compute the aggregated results if more than one file was specified
        # if len(filenames) > 1:
        agg_correct = 0
        agg_id_switch = 0
        agg_miss = 0
        agg_fp = 0
        agg_objects = 0
        agg_motp_dist = 0
        agg_motp_dist_count = 0
        for correct_assignments, id_switches, misses, false_positives, MOTA, MOTP, total_objects, MOTP_dist_cum, MOTP_dist_count in aggregated_results:
            agg_correct += correct_assignments
            agg_id_switch += id_switches
            agg_miss += misses
            agg_fp += false_positives
            agg_objects += total_objects
            agg_motp_dist += MOTP_dist_cum
            agg_motp_dist_count += MOTP_dist_count

        if agg_objects > 0:
            agg_MOTA = 1 - (agg_id_switch + agg_fp + agg_miss)/float(agg_objects)
        else:
            agg_MOTA = -9999999999

        if agg_motp_dist_count > 0:
            agg_MOTP = agg_motp_dist/agg_motp_dist_count
        else:
            agg_MOTP = -9999999999

        # print "\n\n Aggregated results:"
        print "\n\n"
        # Calculate avg dist of followee
        # dist_avg = self.ground_truth_cum_dist/float(self.ground_truth_msg_count)
        # std = math.sqrt(self.ground_truth_M_2n/float(self.ground_truth_msg_count))
        # rospy.loginfo("avg dist of followee is: %.2fm +/-%.2fm", dist_avg, std) # The std will be a little off because of transitions between files but I'll assume that's negligable
        # And other CLEAR MOT stats
        print "Aggregated correct:%d id_switches:%d, misses:%d, FPs:%d, MOTA:%.2f%% MOTP:%.2fm total_objects:%d" % (agg_correct, agg_id_switch, agg_miss, agg_fp, 100*agg_MOTA, agg_MOTP, agg_objects)
        print "\n\n"






    def __del__(self):
        self.close_bags()


    def close_bags(self, not_using_this_var=None):
        self.readbag.close()
        self.savebag.close()       


    def calc_clear_mot(self):
        # For calculating MOTA
        id_switches_count = 0
        false_positives_count = 0
        misses_count = 0
        correct_assignments_count = 0
        total_objects_count = 0
        
        # For calculating MOTP
        MOTP_dist_cum = 0
        MOTP_dist_count = 0

        prev_marker_id = 1

        prev_matched_ids = {}
        ros_bag_not_empty = True
        next_scan_msg = None
        cur_scan_msg_time = None
        next_scan_msg_time = None
        while ros_bag_not_empty:
            # Write the previously saved scan message to the rosbag
            if next_scan_msg:                
                topic, msg, t = next_scan_msg
                next_scan_msg = None
                self.savebag.write(topic, msg, t)
                cur_scan_msg_time = t   

            # Iterate through rosbag until we hit a laser scan message, then break out
            # We also want to populate estimate_people_tracked and ground_truth_people_tracked
            # We assume all people tracks are published once per laser scan
            ground_truth_people_tracked = []
            estimate_people_tracked = [] 
            while True:
                try: 
                    topic, msg, t = next(self.msg_gen)

                    if topic == self.scan_topic:
                        next_scan_msg = (topic, msg, t)
                        next_scan_msg_time = t

                        if self.laser_frame is None:
                            self.laser_frame = msg.header.frame_id

                        break
                    else:
                        # Save message to savebag first
                        self.savebag.write(topic, msg, t)

                        # Append values to ground_truth_people_tracked and estimate_people_tracked                            
                        if topic == "/tf":
                            self.tf_pub.publish(msg)    
                        elif topic == '/ground_truth_people_tracks':
                            ground_truth_people_tracked.append(msg)
                        elif topic == '/people_tracked':
                            estimate_people_tracked_person_array = msg
                            for person in estimate_people_tracked_person_array.people:
                                estimate_people_tracked.append(person)
                except:
                    rospy.loginfo("End of readbag file, shutting down")
                    ros_bag_not_empty = False
                    break

            # On the first iteration of the program, we'll need to fetch two scan messages
            if cur_scan_msg_time == None:
                continue    

            # Remove any ground truth and estimated people that are outside of the (min_angle, max_angle) range
            if self.constrain_to_specific_angle:
                for person in estimate_people_tracked[:]:
                    angle = math.atan2(person.pose.position.y, person.pose.position.x)
                    if angle < self.min_angle or angle > self.max_angle:
                        estimate_people_tracked.remove(person)
                for person in ground_truth_people_tracked[:]:
                    angle = math.atan2(person.pose.position.y, person.pose.position.x)
                    if angle < self.min_angle or angle > self.max_angle:
                        ground_truth_people_tracked.remove(person)                

            # Count the number of tracking objects
            total_objects_count += len(ground_truth_people_tracked)

            # Get some statistics on the distances between the ground truth markers and the laser scanner
            if self.calc_ground_truth_marker_dist_statistics:
                for gt_person in ground_truth_people_tracked:
                    dist = math.sqrt((gt_person.pose.position.x**2. + gt_person.pose.position.y**2.))
                    if self.ground_truth_msg_count is None:
                        self.ground_truth_msg_count = 1
                        self.ground_truth_cum_dist = dist 
                        self.ground_truth_avg_prev = dist
                        self.ground_truth_M_2n = 0
                    else:
                        self.ground_truth_msg_count += 1
                        self.ground_truth_cum_dist += dist
                        avg_new = self.ground_truth_cum_dist/float(self.ground_truth_msg_count)
                        self.ground_truth_M_2n = self.ground_truth_M_2n + (dist - self.ground_truth_avg_prev)*(dist - avg_new); # from: http://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#On-line_algorithm
                        self.ground_truth_avg_prev = avg_new
               
            # Check to see if matches from previous iteration are still valid
            matches = {}
            for prev_est_person_id, prev_gt_person_id in prev_matched_ids.iteritems():
                # See if we can find both ids from previous match
                found_est_person = None
                found_gt_person = None
                for est_person in estimate_people_tracked:
                    if est_person.id == prev_est_person_id:
                        found_est_person = est_person
                        break
                for gt_person in ground_truth_people_tracked:
                    if gt_person.id == prev_gt_person_id:
                        found_gt_person = gt_person
                        break
                if found_est_person and found_gt_person:
                    # We found both ids. Check if they're within the required distance of each other
                    dist = ((found_est_person.pose.position.x - found_gt_person.pose.position.x)**2 + (found_est_person.pose.position.y - found_gt_person.pose.position.y)**2)**(1./2.)
                    if dist < self.max_match_threshold:
                        matches[found_est_person] = found_gt_person

            # Minimum matching between remaining eligable ground truth tracks and estimated tracks
            dist_matrix = []
            # Populate dist_matrix
            for est_person in estimate_people_tracked:
                dist_row = []
                for gt_person in ground_truth_people_tracked:
                    # Calculate distance. But if the estimated_person or the ground_truth_person has already been matched, give them a gigantic distance so they never get matched
                    if est_person not in matches and gt_person not in matches.values():
                        dist = ((est_person.pose.position.x - gt_person.pose.position.x)**2 + (est_person.pose.position.y - gt_person.pose.position.y)**2)**(1./2.)
                    else:
                        dist = 99999.
                    dist_row.append(dist)
                dist_matrix.append(dist_row)
            # Run munkres on dist_matrix
            if dist_matrix:
                munkres = Munkres()
                indexes = munkres.compute(dist_matrix)
                for est_index, gt_index in indexes:
                    if dist_matrix[est_index][gt_index] < self.max_match_threshold:
                        matches[estimate_people_tracked[est_index]] = ground_truth_people_tracked[gt_index]

            # Calculate false positives
            false_positives_set = set()
            for est_person in estimate_people_tracked:
                if est_person not in matches:
                    false_positives_set.add(est_person)
                    false_positives_count += 1

            # Calculate misses
            misses_set = set()
            for gt_person in ground_truth_people_tracked:
                if gt_person not in matches.values():
                    misses_set.add(gt_person)
                    misses_count += 1

            # Calculate id switches and correct assignments     
            id_switches_set = set()      
            for est_person, gt_person in matches.items():
                if est_person.id in prev_matched_ids: 
                    if prev_matched_ids[est_person.id] == gt_person.id: 
                        # same matching as last time!
                        correct_assignments_count += 1   
                    else: 
                        # est_person is matched to a different gt_person from prev_matched_ids
                        id_switches_set.add(est_person)
                        id_switches_count += 1
                elif gt_person.id in prev_matched_ids.values(): 
                    # new est_person is matched to a gt_person from prev_matched_ids
                    id_switches_set.add(gt_person)
                    id_switches_count += 1
                else:
                    # new est_person and new gt_person who haven't been matched before
                    correct_assignments_count += 1

            # Calculate MOTP
            for est_person, gt_person in matches.items():
                dist = ((est_person.pose.position.x - gt_person.pose.position.x)**2 + (est_person.pose.position.y - gt_person.pose.position.y)**2)**(1./2.)
                MOTP_dist_cum += dist
                MOTP_dist_count += 1

            # Clear any key-value pair from prev_matched_ids which has a key or value in matches
            # So prev_matches can be updated with new matches without double-storing any est_persons or gt_persons
            # Any key-value pair not in matches can stay in prev_matched_ids
            keys_to_delete = set()
            for prev_est_person_id, prev_gt_person_id in prev_matched_ids.items():
                for est_person, gt_person in matches.items():
                    if prev_est_person_id == est_person.id or prev_gt_person_id == gt_person.id:
                        keys_to_delete.add(prev_est_person_id)
            for key in keys_to_delete:
                del prev_matched_ids[key]
            # Add any new matchings to prev_match_ids for next iteration
            for est_person, gt_person in matches.items():
                prev_matched_ids[est_person.id] = gt_person.id # Note: we're doing the ids here because they should stay constant between cycles                

            # Draw markers of matchings and scores for rviz and save to the rosbag
            write_time = (next_scan_msg_time - cur_scan_msg_time)/2. + cur_scan_msg_time
            self.save_markers_to_rosbag(write_time, matches, id_switches_set, false_positives_set, misses_set, id_switches_count, false_positives_count, misses_count, correct_assignments_count)

        # Finished going through all messages in rosbag
        self.close_bags()

        # Cacluate statistics on the distances between the ground truth markers and the laser scanner
        # Basically only used to calculate the average distance to the followee when we were tracking only one person
        # dist_avg = self.ground_truth_cum_dist/float(self.ground_truth_msg_count)
        # std = math.sqrt(self.ground_truth_M_2n/float(self.ground_truth_msg_count))
        # rospy.loginfo("avg dist of followee is: %.2fm +/-%.2fm", dist_avg, std)

        # Cacluate cumilative CLEAR MOT statistics
        if total_objects_count > 0:
            MOTA = 1 - (id_switches_count + false_positives_count + misses_count)/float(total_objects_count)
        else:
            MOTA = -9999999
        if MOTP_dist_count > 0:
            MOTP = MOTP_dist_cum/MOTP_dist_count
        else:
            MOTP = -9999999

        return  correct_assignments_count, id_switches_count, misses_count, false_positives_count, MOTA, MOTP, total_objects_count, MOTP_dist_cum, MOTP_dist_count


    def save_markers_to_rosbag(self, write_time, matches, id_switches_set, false_positives_set, misses_set, id_switches_count, false_positives_count, misses_count, correct_assignments_count):
        # Draw a green arrow marker to show matches
        marker_id = 0
        correct_num = correct_assignments_count
        for est_person, gt_person in matches.items():
            # Draw an arrow between matches for viewing in Rviz
            m = Marker()
            m.header.frame_id = self.laser_frame
            m.header.stamp = write_time                        
            m.id = marker_id
            marker_id += 1
            m.ns = "clear_mot"
            m.type = Marker.ARROW
            est_point = est_person.pose.position
            est_point.z = 1.1
            gt_point = gt_person.pose.position    
            gt_point.z = 1.1
            m.points.append(gt_point)
            m.points.append(est_point)
            m.color.r = 0.
            m.color.g = 1.
            m.color.b = 0.                                       
            m.color.a = 1.
            m.scale.x = 0.05
            m.scale.y = 0.1
            self.savebag.write("/visualization_marker", m, write_time)

            # Draw a text marker too
            if est_person not in id_switches_set and gt_person not in id_switches_set:
                m = Marker()
                m.header.frame_id = self.laser_frame
                m.header.stamp = write_time            
                m.id = marker_id
                marker_id += 1
                m.ns = "clear_mot"
                m.type = Marker.TEXT_VIEW_FACING
                m.text = "correct: " + str(correct_num)
                correct_num -= 1
                m.pose.position.x = est_person.pose.position.x
                m.pose.position.y = est_person.pose.position.y
                m.pose.position.z = 1.5
                m.color.r = 1.
                m.color.g = 1.
                m.color.b = 1.                                       
                m.color.a = 1.
                m.scale.z = 0.17
                self.savebag.write("/visualization_marker", m, write_time)

        # Draw a text marker to show false positives
        fp_num = false_positives_count
        for est_person in false_positives_set:
            m = Marker()
            m.header.frame_id = self.laser_frame
            m.header.stamp = write_time                        
            m.id = marker_id
            marker_id += 1
            m.ns = "clear_mot"
            m.type = Marker.TEXT_VIEW_FACING
            m.text = "fp: " + str(fp_num) 
            fp_num -= 1
            m.pose.position.x = est_person.pose.position.x
            m.pose.position.y = est_person.pose.position.y
            m.pose.position.z = 1.5
            m.color.r = 1.
            m.color.g = 1.
            m.color.b = 1.                                       
            m.color.a = 1.
            m.scale.z = 0.17
            self.savebag.write("/visualization_marker", m, write_time)

        # Draw a text marker to show misses
        miss_num = misses_count
        for gt_person in misses_set:
            m = Marker()
            m.header.frame_id = self.laser_frame
            m.header.stamp = write_time                        
            m.id = marker_id
            marker_id += 1
            m.ns = "clear_mot"
            m.type = Marker.TEXT_VIEW_FACING
            m.text = "miss: " + str(miss_num)
            miss_num -= 1
            m.pose.position.x = gt_person.pose.position.x
            m.pose.position.y = gt_person.pose.position.y
            m.pose.position.z = 1.5
            m.color.r = 1.
            m.color.g = 1.
            m.color.b = 1.                                       
            m.color.a = 1.
            m.scale.z = 0.17
            self.savebag.write("/visualization_marker", m, write_time)

        # Display id switches to Rviz
        id_s_num = id_switches_count
        for person in id_switches_set:
            m = Marker()
            m.header.frame_id = self.laser_frame
            m.header.stamp = write_time                        
            m.id = marker_id
            marker_id += 1           
            m.ns = "clear_mot" 
            m.type = Marker.TEXT_VIEW_FACING
            m.text = "id switch: " + str(id_s_num) 
            id_s_num -= 1
            m.pose.position.x = person.pose.position.x
            m.pose.position.y = person.pose.position.y
            m.pose.position.z = 1.5
            m.color.r = 1.
            m.color.g = 1.
            m.color.b = 1.                                       
            m.color.a = 1.
            m.scale.z = 0.17
            self.savebag.write("/visualization_marker", m, write_time) 

        # Draw a line showing viable field of view
        if self.constrain_to_specific_angle:
            m = Marker()
            m.header.frame_id = self.laser_frame
            m.id = marker_id
            marker_id += 1
            m.ns = "clear_mot"
            m.type = Marker.LINE_STRIP
            m.points.append(Point(10*math.cos(self.max_angle), 10*math.sin(self.max_angle), 0))
            m.points.append(Point(0,0,0))
            m.points.append(Point(10*math.cos(self.min_angle), 10*math.sin(self.min_angle), 0))
            m.color.r = 0.
            m.color.g = 0.
            m.color.b = 1.                                       
            m.color.a = 0.5
            m.scale.x = 0.005
            self.savebag.write("/visualization_marker", m, write_time)


        # Clear previous markers
        if self.prev_marker_id > marker_id:
            for m_id in xrange(marker_id, self.prev_marker_id):
                m = Marker()
                m.header.frame_id = self.laser_frame
                m.header.stamp = write_time                            
                m.ns = "clear_mot"
                m.id = m_id
                m.action = m.DELETE   
                self.savebag.write("/visualization_marker", m, write_time)
        self.prev_marker_id = marker_id


if __name__ == '__main__':
    # Read-in the bags to be evaluated and saved. Multiple readbags and savebags can be used and their statistics will be aggregated
    filenames = []
    for i in xrange(1, 99):
        next_readbag_filename = rospy.get_param("readbag_filename_" + str(i), None)
        next_savebag_filename = rospy.get_param("savebag_filename_" + str(i), None)
        if next_readbag_filename is not None and next_savebag_filename is not None:
            filenames.append((next_readbag_filename, next_savebag_filename))
        else:
            break

    ccm = ClearMot(filenames)




