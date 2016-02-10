/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include <leg_tracker/laser_processor.h>
#include <leg_tracker/calc_cluster_features.h>

#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/ml.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/transform_listener.h>

#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <math.h>       /* atan2 */


// Reads in a rosbag, finds all scan clusters lying within user-specified bounding box (or alternatively min/max angles), marks those clusters as positive examples and saves the result to a new rosbag
// Used to quickly get positive examples for training the leg_tracker


using namespace std;


#define PI 3.14159265

class ExtractPositiveTrainingClusters
{
public:
  tf::TransformListener tfl_;
  string laser_frame_;
  string scan_topic_;

  int feat_count_;

  ros::NodeHandle nh_;

  std::string save_bag_file_;
  std::string load_bag_file_;  

  double cluster_dist_euclid_;
  double min_points_per_cluster_;

  // to describe bounding box containing positive clusters
  bool use_bounding_box_;
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  
  // to describe scan angles containing positive clusters
  int min_angle_;
  int max_angle_;
  int max_dist_;


  ExtractPositiveTrainingClusters() 
  {
    // Get ROS parameters  
    ros::NodeHandle local_nh("~"); // to get private parameters
    
    if (!local_nh.getParam("load_bag_file", load_bag_file_))
      ROS_ERROR("Couldn't get bag_load_file from ros param server");
    if (!local_nh.getParam("save_bag_file", save_bag_file_))
      ROS_ERROR("Couldn't get bag_save_file from ros param server");
    if (!local_nh.getParam("scan_topic", scan_topic_))
      ROS_ERROR("Couldn't get scan_topic from ros param server");
    if (!local_nh.getParam("laser_frame", laser_frame_))
      ROS_ERROR("Couldn't get laser_frame from ros param server");
    if (!nh_.getParam("cluster_dist_euclid", cluster_dist_euclid_))
      ROS_ERROR("Couldn't get cluster_dist_euclid from ros param server");   
    if (!nh_.getParam("min_points_per_cluster", min_points_per_cluster_))
      ROS_ERROR("Couldn't get min_points_per_cluster from ros param server");      
    
    if (!local_nh.getParam("x_min", x_min_) or
        !local_nh.getParam("x_max", x_max_) or
        !local_nh.getParam("y_min", y_min_) or
        !local_nh.getParam("y_max", y_max_))                        
    {
      ROS_INFO("Couldn't get bounding box for positive clusters. Assuming you've specified a min/max scan angle and a max distance instead.");
      use_bounding_box_ = false;
    }
    else
    {
      use_bounding_box_ = true;
    }
    
    if (!local_nh.getParam("min_angle", min_angle_) or
        !local_nh.getParam("max_angle", max_angle_) or
        !local_nh.getParam("max_dist", max_dist_))                       
    {
      if (use_bounding_box_)
      {
        ROS_INFO("Couldn't get min/max scan angle for positive clusters. Assuming you've specified a bounding box instead.");
      }
      else
      {
        ROS_ERROR("Couldn't get bounding box or scan min/max angle for positive clusters");
      }
    }
  }


  ~ExtractPositiveTrainingClusters()
  {
  }


  void extract()
  {
    // Open rosbag we'll be saving to
    rosbag::Bag save_bag;
    save_bag.open(save_bag_file_.c_str(), rosbag::bagmode::Write);

    // Open rosbag we'll be loading from
    rosbag::Bag load_bag;
    load_bag.open(load_bag_file_.c_str(), rosbag::bagmode::Read);
    
    // Iterate through all scan messages in the loaded rosbag
    std::vector<std::string> topics;
    topics.push_back(std::string(scan_topic_));
    rosbag::View view(load_bag, rosbag::TopicQuery(topics)); 
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      sensor_msgs::LaserScan::ConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();
      if (scan != NULL)
      {
        laser_processor::ScanProcessor processor(*scan);
        processor.splitConnected(cluster_dist_euclid_);
        processor.removeLessThan(min_points_per_cluster_);

        geometry_msgs::PoseArray leg_cluster_positions;
        leg_cluster_positions.header.frame_id = "laser_frame";

        for (list<laser_processor::SampleSet*>::iterator i = processor.getClusters().begin();
             i != processor.getClusters().end();
             i++)
        {
          // Only use clusters that are in the specified positive cluster area
          tf::Point cluster_position = (*i)->getPosition();

          double x_pos = cluster_position[0];
          double y_pos = cluster_position[1];
          double angle = atan2(y_pos,x_pos) * 180 / PI;
          double dist_abs = sqrt(x_pos*x_pos + y_pos*y_pos);

          bool in_bounding_box = use_bounding_box_ and x_pos > x_min_ and x_pos < x_max_ and y_pos > y_min_ and y_pos < y_max_;
          bool in_scan_angles = !use_bounding_box_ and angle > min_angle_ and angle < max_angle_ and dist_abs < max_dist_;
          if (in_bounding_box or in_scan_angles) 
          {          
            geometry_msgs::Pose new_leg_cluster_position;
            new_leg_cluster_position.position.x = cluster_position[0];
            new_leg_cluster_position.position.y = cluster_position[1];
            leg_cluster_positions.poses.push_back(new_leg_cluster_position);
          }
        }
        if (!leg_cluster_positions.poses.empty())  // at least one leg has been found in current scan
        {
          // Save position of leg to be used later for training 
          save_bag.write("/leg_cluster_positions", ros::Time::now(), leg_cluster_positions); 

          // Save an identical scan to one we recieved but with a different topic and reference frame.
          sensor_msgs::LaserScan new_scan; 
          new_scan.header.frame_id = "laser_frame"; 
          new_scan.angle_min = scan->angle_min;
          new_scan.angle_max = scan->angle_max;
          new_scan.angle_increment = scan->angle_increment;
          new_scan.range_min = scan->range_min;
          new_scan.range_max = scan->range_max;
          new_scan.ranges = scan->ranges;
          save_bag.write("/training_scan", ros::Time::now(), new_scan);

          // Save a marker of the position of the cluster we extracted. Just used so we can playback the rosbag file and visually verify the correct clusters have been extracted
          visualization_msgs::MarkerArray ma;
          int MAX_NUM_MARKERS = 10;
          for (int i = 0;
              i < leg_cluster_positions.poses.size();
              i++)
          {
            // display cluster to view in rviz
            visualization_msgs::Marker m;
            m.header.frame_id = "laser_frame";
            m.ns = "LEGS";
            m.id = i;
            m.type = m.SPHERE;
            m.pose.position.x = leg_cluster_positions.poses[i].position.x;
            m.pose.position.y = leg_cluster_positions.poses[i].position.y;
            m.pose.position.z = 0.1;
            m.scale.x = .2;
            m.scale.y = .2;
            m.scale.z = .2;
            m.color.a = 1;
            m.lifetime = ros::Duration(0.2);
            m.color.b = 0.0;
            m.color.r = 1.0;
            ma.markers.push_back(m);
          }
          for (int i = leg_cluster_positions.poses.size();  // clearing any lingering markers from Rviz
              i <= MAX_NUM_MARKERS;
              i++)
          {
            visualization_msgs::Marker m;
            m.header.frame_id = "laser_frame";
            m.ns = "LEGS";
            m.id = i;
            m.action = visualization_msgs::Marker::DELETE;
            ma.markers.push_back(m);
          }
          save_bag.write("/visualization_marker_array", ros::Time::now(), ma);
        }
      }
    }
    load_bag.close();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv,"extract_positive_leg_clusters");
  ExtractPositiveTrainingClusters eptc;
  eptc.extract();
  return 0;
}

