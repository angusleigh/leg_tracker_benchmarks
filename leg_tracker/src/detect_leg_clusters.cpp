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

#include <leg_tracker/laser_processor.h>
#include <leg_tracker/calc_cluster_features.h>

#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/ml.h>

#include <sensor_msgs/LaserScan.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <visualization_msgs/Marker.h>

#include <algorithm>

// Custom messages
#include <leg_tracker/Leg.h>
#include <leg_tracker/LegArray.h>


#include <time.h>
#define BILLION  1000000000L;



using namespace std;

struct Cmp
{
    bool operator ()(const pair<leg_tracker::Leg, float> &a, const pair<leg_tracker::Leg, float> &b)
    {
        return a.second < b.second;
    }
};


class DetectLegClusters
{
public:
  tf::TransformListener tfl_;

  CvRTrees forest;

  int feat_count_;

  int scan_num_;
  bool use_scan_header_stamp_for_tfs_;
  ros::Time latest_scan_header_stamp_with_tf_available_;

  ros::NodeHandle nh_;
  ros::Publisher markers_pub_;
  ros::Publisher detected_leg_clusters_pub_;
  ros::Subscriber scan_sub_;

  string fixed_frame_;
  
  double detection_threshold_;
  double cluster_dist_euclid_;
  int min_points_per_cluster_;  
  double max_detect_distance_;
  double marker_display_lifetime_;

  int num_prev_markers_published_;

  int max_detected_clusters_;

  DetectLegClusters():
  scan_num_(0),
  num_prev_markers_published_(0)
  {  
    // Get ROS parameters  
    std::string forest_file;
    std::string scan_topic;
    if (!nh_.getParam("forest_file", forest_file))
      ROS_ERROR("ERROR! Could not get random forest filename");
    nh_.param("scan_topic", scan_topic, std::string("scan"));
    nh_.param("fixed_frame", fixed_frame_, std::string("odom"));
    nh_.param("detection_threshold", detection_threshold_, -1.0);
    nh_.param("cluster_dist_euclid", cluster_dist_euclid_, 0.13);
    nh_.param("min_points_per_cluster", min_points_per_cluster_, 3);                
    nh_.param("max_detect_distance", max_detect_distance_, 10.0);   
    nh_.param("marker_display_lifetime", marker_display_lifetime_, 0.2);   
    nh_.param("use_scan_header_stamp_for_tfs", use_scan_header_stamp_for_tfs_, false);
    nh_.param("max_detected_clusters", max_detected_clusters_, -1);

    // Print back
    ROS_INFO("forest_file: %s", forest_file.c_str());
    ROS_INFO("scan_topic: %s", scan_topic.c_str());
    ROS_INFO("fixed_frame: %s", fixed_frame_.c_str());
    ROS_INFO("detection_threshold: %.2f", detection_threshold_);
    ROS_INFO("cluster_dist_euclid: %.2f", cluster_dist_euclid_);
    ROS_INFO("min_points_per_cluster: %d", min_points_per_cluster_);
    ROS_INFO("max_detect_distance: %.2f", max_detect_distance_);    
    ROS_INFO("marker_display_lifetime: %.2f", marker_display_lifetime_);

    // Load random forst
    forest.load(forest_file.c_str());
    feat_count_ = forest.get_active_var_mask()->cols;

    latest_scan_header_stamp_with_tf_available_ = ros::Time::now();

    // ROS subscribers + publishers
    scan_sub_ =  nh_.subscribe(scan_topic, 10, &DetectLegClusters::laserCallback, this);
    markers_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 200);
    detected_leg_clusters_pub_ = nh_.advertise<leg_tracker::LegArray>("detected_leg_clusters", 200);
  }


  // Called every time a laser scan is published.
  // It clusters the scan according to euclidian distance, predicts the confidence that each cluster is a human leg and publishes the results.
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {      
    struct timespec tic, toc;
    clock_gettime(CLOCK_REALTIME, &tic);

    
    laser_processor::ScanProcessor processor(*scan); 
    processor.splitConnected(cluster_dist_euclid_);        
    processor.removeLessThan(min_points_per_cluster_);    

    CvMat* tmp_mat = cvCreateMat(1, feat_count_, CV_32FC1); 
    
    leg_tracker::LegArray detected_leg_clusters;
    detected_leg_clusters.header.frame_id = scan->header.frame_id;
    detected_leg_clusters.header.stamp = scan->header.stamp;

    // Find out the time that should be used for tfs
    bool transform_available;
    ros::Time tf_time;
    // Use time from scan header
    if (use_scan_header_stamp_for_tfs_)
    {
      tf_time = scan->header.stamp;

      try
      {
        tfl_.waitForTransform(fixed_frame_, scan->header.frame_id, tf_time, ros::Duration(1.0));
        transform_available = tfl_.canTransform(fixed_frame_, scan->header.frame_id, tf_time);
      }
      catch(tf::TransformException ex)
      {
        ROS_INFO("Detect_leg_clusters: No tf available");
        transform_available = false;
      }
    }
    else
    {
      // Otherwise just use the latest tf available
      tf_time = ros::Time(0);
      transform_available = tfl_.canTransform(fixed_frame_, scan->header.frame_id, tf_time);
    }
    
    // These are needed so we can publish the closest <max_detected_clusters_>
    pair<leg_tracker::Leg, float> leg_and_rel_dist_pair;
    set <pair<leg_tracker::Leg, float>, Cmp> leg_and_rel_dist_set;

    if (transform_available)
    {
      // Iterate through all clusters
      for (list<laser_processor::SampleSet*>::iterator cluster = processor.getClusters().begin();
       cluster != processor.getClusters().end();
       cluster++)
      {   
        // Get position of cluster in laser frame
        tf::Stamped<tf::Point> position((*cluster)->getPosition(), tf_time, scan->header.frame_id);
        float rel_dist = pow(position[0]*position[0] + position[1]*position[1], 1./2.);
        
        // Only consider clusters within max_distance. 
        if (rel_dist < max_detect_distance_)
        {
          // Classify cluster
          vector<float> f = calcClusterFeatures(*cluster, *scan);
          for (int k = 0; k < feat_count_; k++)
          {
            tmp_mat->data.fl[k] = (float)(f[k]);
          }
          float probability_of_leg = forest.predict_prob( tmp_mat );

          // Publish only clusters that have a confidence greater than detection_threshold_                 
          if (probability_of_leg > detection_threshold_)
          { 
            // Transform cluster position to fixed frame
            bool transform_successful_2 = false;
            try
            {
              tfl_.transformPoint(fixed_frame_, position, position);
              transform_successful_2 = true;
            }
            catch (tf::TransformException ex)
            {
              ROS_INFO("Detect_leg_clusters: tf error at location 2");
              transform_successful_2 = false;
            }

            if (transform_successful_2)
            {  
              // Publish to detected_leg_clusters topic
              leg_tracker::Leg new_leg_cluster;
              new_leg_cluster.position.x = position[0];
              new_leg_cluster.position.y = position[1];
              new_leg_cluster.confidence = probability_of_leg;
              pair<leg_tracker::Leg, float> new_pair = make_pair(new_leg_cluster, rel_dist);
              leg_and_rel_dist_set.insert(new_pair);
              // detected_leg_clusters.legs.push_back(new_leg_cluster); 
            }
          }
        }
      }     
    }    
    else
    {
      ROS_INFO("Not publishing detected clusters because no tf was available at scan header time");
    }

    // // Publish detect legs to rviz
    // if (max_detected_clusters_ < 0)
    // {
      // No limit on the detected clusters to publish so publish them all
    int clusters_published_counter = 0;
    int id_num = 1;      
    std::set<pair<leg_tracker::Leg, float> >::iterator it;
    for (it = leg_and_rel_dist_set.begin(); it != leg_and_rel_dist_set.end(); ++it)
    {
      pair<leg_tracker::Leg, float> leg_dist_pair = *it;
      leg_tracker::Leg leg_cluster = leg_dist_pair.first;
      detected_leg_clusters.legs.push_back(leg_cluster);
      clusters_published_counter++;

      // Publish marker
      visualization_msgs::Marker m;
      m.header.stamp = scan->header.stamp;
      m.header.frame_id = fixed_frame_;
      m.ns = "LEGS";
      m.id = id_num++;
      m.type = m.SPHERE;
      m.pose.position.x = leg_cluster.position.x ;
      m.pose.position.y = leg_cluster.position.y;
      m.pose.position.z = 0.2;
      m.scale.x = 0.13;
      m.scale.y = 0.13;
      m.scale.z = 0.13;
      m.color.a = 1;
      // m.lifetime = ros::Duration(marker_display_lifetime_);
      m.color.r = 0;
      m.color.g = leg_cluster.confidence;
      m.color.b = leg_cluster.confidence;
      markers_pub_.publish(m);

      if (clusters_published_counter == max_detected_clusters_)
        break;
    }

    // Clear remaining markers in Rviz
    for (int id_num_diff = num_prev_markers_published_ - id_num - 1; id_num_diff >= 0; id_num_diff--)
    {
      visualization_msgs::Marker m;
      m.header.stamp = scan->header.stamp;
      m.header.frame_id = fixed_frame_;
      m.ns = "LEGS";
      m.id = id_num_diff + id_num;
      m.action = m.DELETE;
      markers_pub_.publish(m);
    }
    num_prev_markers_published_ = id_num;   

    detected_leg_clusters_pub_.publish(detected_leg_clusters);
    cvReleaseMat(&tmp_mat);

    clock_gettime(CLOCK_REALTIME, &toc);
    double elapsed = ( toc.tv_sec - tic.tv_sec ) + (double)( toc.tv_nsec - tic.tv_nsec ) / (double)BILLION;
    // printf("detect: %.3f\n", elapsed);
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "detect_leg_clusters");
  DetectLegClusters dlc;
  ros::spin();
  return 0;
}

