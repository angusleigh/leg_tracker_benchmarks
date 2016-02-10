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


#include <leg_tracker/laser_processor.h>
#include <leg_tracker/calc_cluster_features.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/ml.h"

#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/PoseArray.h>


// Trains the leg_detector to classify scan clusters as legs/not legs
// Reads in rosbags: positive examples are obtained from rosbags annotated by extract_positive_training_clusters 
//                   negative examples are obtained from rosbags where the laser was moving around an empty room
// Output is an yaml file which configures an OpenCV random forest classifier


using namespace std;

class TrainLegDetector
{
public:
  char* scan_topic;

  CvRTrees forest;

  CvSVM svm_;

  int feat_count_;
  
  float training_error_;
  
  double cluster_dist_euclid_;
  int min_points_per_cluster_;  
  int undersample_negative_factor_;
  string positive_leg_cluster_positions_topic_;
  string save_file_;

  TrainLegDetector(ros::NodeHandle nh): 
    feat_count_(0)
  {
    // Get params from ROS param server. 
    nh.param("cluster_dist_euclid", cluster_dist_euclid_, 0.13);
    nh.param("min_points_per_cluster", min_points_per_cluster_, 3);
    nh.param("undersample_negative_factor", undersample_negative_factor_, 50);
    nh.param("positive_leg_cluster_positions_topic", positive_leg_cluster_positions_topic_, std::string("/leg_cluster_positions"));
    // Mandatory param:
    if (!nh.getParam("save_file", save_file_))
      ROS_ERROR("Couldn't get save_file from ros param server"); 

    // Print back params:
    printf("\nParameters: \n");
    printf("cluster_dist_euclid:%.2fm \n", cluster_dist_euclid_);
    printf("min_points_per_cluster:%i \n", min_points_per_cluster_);
    printf("undersample_negative_factor:%i \n", undersample_negative_factor_);
    printf("positive_leg_cluster_positions_topic:%s \n", positive_leg_cluster_positions_topic_.c_str());
    printf("\n");
  }


  // Load scan messages and positive cluster position markers from the rosbag_file,
  // separate the scan into clusters, figure out which clusters lie near a positive marker,
  // calcualte features on those clusters, save features from each cluster to <data>.
  void loadPosData(
    const char* rosbag_file, 
    const char* scan_topic, 
    vector< vector<float> > &data
    )
  {
    rosbag::Bag bag;
    bag.open(rosbag_file, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string(scan_topic)); 
    topics.push_back(std::string(positive_leg_cluster_positions_topic_)); 
    rosbag::View view(bag, rosbag::TopicQuery(topics)); 

    geometry_msgs::PoseArray positive_clusters;

    int message_num = 0;
    int initial_pos_data_size = (int)data.size();
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      geometry_msgs::PoseArray::ConstPtr pose_array_msg = m.instantiate<geometry_msgs::PoseArray>();
      if (pose_array_msg != NULL)
      {
        positive_clusters = *pose_array_msg;
      }

      sensor_msgs::LaserScan::ConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();
      if (scan != NULL and positive_clusters.poses.size())
      {  
        laser_processor::ScanProcessor processor(*scan);
        processor.splitConnected(cluster_dist_euclid_);
        processor.removeLessThan(min_points_per_cluster_);
 
        for (list<laser_processor::SampleSet*>::iterator i = processor.getClusters().begin();
             i != processor.getClusters().end();
             i++)
        {
          tf::Point cluster_position = (*i)->getPosition();

          for (int j = 0; 
                   j < positive_clusters.poses.size();
                   j++)
          {
            // Only use clusters which are close to a "marker"
            double dist_x = positive_clusters.poses[j].position.x - cluster_position[0],
                   dist_y = positive_clusters.poses[j].position.y - cluster_position[1],             
                   dist_abs = sqrt(dist_x*dist_x + dist_y*dist_y);
            if (dist_abs < 0.0001)
            {
              data.push_back(calcClusterFeatures(*i, *scan));
              break;                                           
            }
          }
        }
        message_num++;
      } 
    }
    bag.close();

    printf("\t Got %i scan messages, %i samples, from %s  \n",message_num, (int)data.size() - initial_pos_data_size, rosbag_file);
  } 


  // Load scan messages from the rosbag_file, separate into clusters, 
  // calcualte features on those clusters, save features from each cluster to <data>.
  void loadNegData(
    const char* rosbag_file, 
    const char* scan_topic, 
    vector< vector<float> > &data
    )
  {
    rosbag::Bag bag;
    bag.open(rosbag_file, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string(scan_topic));
    rosbag::View view(bag, rosbag::TopicQuery(topics)); 

    int message_num = 0;
    int initial_neg_data_size = (int)data.size();
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      sensor_msgs::LaserScan::ConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();
      if (scan != NULL)
      {
        laser_processor::ScanProcessor processor(*scan);
        processor.splitConnected(cluster_dist_euclid_);
        processor.removeLessThan(min_points_per_cluster_);
 
        for (list<laser_processor::SampleSet*>::iterator i = processor.getClusters().begin();
             i != processor.getClusters().end();
             i++)
        {
          if (rand() % undersample_negative_factor_ == 0) // one way of undersampling the negative class
            data.push_back(calcClusterFeatures(*i, *scan));                 
        }
        message_num++;
      } 
    }
    bag.close();

    printf("\t Got %i scan messages, %i samples, from %s  \n",message_num, (int)data.size() - initial_neg_data_size, rosbag_file);
  } 


  // Train the classifier using the positive and negative training data
  // We'll use a random forest classifier 
  void train(
    const vector< vector<float> > &train_pos_data, 
    const vector< vector<float> > &train_neg_data
    )
  {
    int sample_size = train_pos_data.size() + train_neg_data.size();
    feat_count_ = train_pos_data[0].size();

    CvMat* cv_data = cvCreateMat( sample_size, feat_count_, CV_32FC1);
    CvMat* cv_resp = cvCreateMat( sample_size, 1, CV_32S);

    // Put positive data in opencv format.
    int j = 0;
    for (vector< vector<float> >::const_iterator i = train_pos_data.begin();
         i != train_pos_data.end();
         i++)
    {
      float* data_row = (float*)(cv_data->data.ptr + cv_data->step*j);
      for (int k = 0; k < feat_count_; k++)
        data_row[k] = (*i)[k];
      
      cv_resp->data.i[j] = 1;
      j++;
    }

    // Put negative data in opencv format.
    for (vector< vector<float> >::const_iterator i = train_neg_data.begin();
         i != train_neg_data.end();
         i++)
    {
      float* data_row = (float*)(cv_data->data.ptr + cv_data->step*j);
      for (int k = 0; k < feat_count_; k++)
        data_row[k] = (*i)[k];
      
      cv_resp->data.i[j] = -1;
      j++;
    }

    CvMat* var_type = cvCreateMat( 1, feat_count_ + 1, CV_8U );
    cvSet( var_type, cvScalarAll(CV_VAR_ORDERED));
    cvSetReal1D( var_type, feat_count_, CV_VAR_CATEGORICAL );
    
    // Random forest training parameters
    // One important parameter not set here is undersample_negative_factor.
    // I tried to keep the params similar to the defaults in scikit-learn
    float priors[] = {1.0, 1.0};

    CvRTParams fparam(
      10000,              // max depth of tree
      2,                  // min sample count to split tree
      0,                  // regression accuracy (?)
      false,              // use surrogates (?)
      1000,               // max categories
      priors,             // priors
      false,              // calculate variable importance 
      2,                  // number of active vars for each tree node (default from scikit-learn is: (int)round(sqrt(feat_count_))
      100,                // max trees in forest (default of 10 from scikit-learn does worse)
      0.001f,             // forest accuracy (sufficient OOB error)
      CV_TERMCRIT_ITER    // termination criteria. CV_TERMCRIT_ITER = once we reach max number of forests
      ); 
       
    forest.train( 
      cv_data,                // train data 
      CV_ROW_SAMPLE,          // tflag
      cv_resp,                // responses (i.e. labels)
      0,                      // varldx (?)
      0,                      // sampleldx (?)
      var_type,               // variable type 
      0,                      // missing data mask
      fparam                  // parameters 
      );                

    training_error_ = 100.0*forest.get_train_error();

    cvReleaseMat(&cv_data);
    cvReleaseMat(&cv_resp);
    cvReleaseMat(&var_type);
  }

  // Test the classifier using pos_data and neg_data
  // Results return in correct_pos and correct_neg
  void test(
    const vector< vector<float> > &pos_data, 
    const vector< vector<float> > &neg_data,
    int &correct_pos,
    int &correct_neg
    )
  {
    CvMat* tmp_mat = cvCreateMat(1,feat_count_,CV_32FC1);

    // test on positive examples
    for (vector< vector<float> >::const_iterator i = pos_data.begin();
         i != pos_data.end();
         i++)
    {
      for (int k = 0; k < feat_count_; k++)
        tmp_mat->data.fl[k] = (float)((*i)[k]);
      if (forest.predict( tmp_mat) > 0)
        correct_pos++;
    }

    // test on negative examples
    for (vector< vector<float> >::const_iterator i = neg_data.begin();
         i != neg_data.end();
         i++)
    {
      for (int k = 0; k < feat_count_; k++)
        tmp_mat->data.fl[k] = (float)((*i)[k]);
      if (forest.predict( tmp_mat ) < 0)
        correct_neg++;
    }

    cvReleaseMat(&tmp_mat);
  }

  // Perform cross validation
  // Results returned in correct_pos, correct_neg, total_pos and total_neg 
  void crossValidate(
    const vector< vector<float> > &pos_data, 
    const vector< vector<float> > &neg_data,
    int cross_val_folds,
    int &correct_pos,
    int &correct_neg,
    int &total_pos,
    int &total_neg
    )
  {
    for (int fold = 0; fold < cross_val_folds; fold++) 
    {
      // printf("  Cross validation fold %i \n", fold);
      
      // Populate current fold's train and test datasets
      vector< vector<float> > cross_val_train_pos_data;
      vector< vector<float> > cross_val_train_neg_data;
      vector< vector<float> > cross_val_test_pos_data;
      vector< vector<float> > cross_val_test_neg_data;
      // Load positive training and test data
      int test_indx_low = (int)pos_data.size()*(float(fold)/float(cross_val_folds));
      int test_indx_high = (int)pos_data.size()*(float(fold+1)/float(cross_val_folds));
      for (int i = 0; i < pos_data.size(); i++)
      {
        if (i < test_indx_low or i > test_indx_high)
          cross_val_train_pos_data.push_back(pos_data[i]);
        else
          cross_val_test_pos_data.push_back(pos_data[i]);
      }
      // Load negative training and test data
      test_indx_low = (int)neg_data.size()*(float(fold)/float(cross_val_folds));
      test_indx_high = (int)neg_data.size()*(float(fold+1)/float(cross_val_folds));
      for (int i = 0; i < neg_data.size(); i++)
      {
        if (i < test_indx_low or i > test_indx_high)
          cross_val_train_neg_data.push_back(neg_data[i]);
        else
          cross_val_test_neg_data.push_back(neg_data[i]);
      }

      // Evaluate current fold
      // printf("    Positive training samples on this fold: %i, Negative: %i \n", (int)cross_val_train_pos_data.size(), (int)cross_val_train_neg_data.size());
      train(cross_val_train_pos_data, cross_val_train_neg_data);
      int correct_pos_fold = 0;
      int correct_neg_fold = 0;
      test(cross_val_test_pos_data, cross_val_test_neg_data, correct_pos_fold, correct_neg_fold);
      correct_pos += correct_pos_fold;
      correct_neg += correct_neg_fold;
      total_pos += (int)cross_val_test_pos_data.size();
      total_neg += (int)cross_val_test_neg_data.size();
    }
  }


  void save()
  {
    forest.save(save_file_.c_str());
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv,"train_leg_detector");

  ros::NodeHandle nh;

  TrainLegDetector tld(nh);

  // Parse command line arguements and load data 
  printf("\nLoading data...\n");
  vector< vector<float> > train_pos_data;
  vector< vector<float> > train_neg_data;
  vector< vector<float> > test_neg_data;
  vector< vector<float> > test_pos_data;
  for (int i = 0; i < argc; i++) 
  {
    if (!strcmp(argv[i],"--pos"))
    {
      char* rosbag_file = argv[++i]; 
      char* scan_topic = argv[++i];
      tld.loadPosData(rosbag_file, scan_topic, train_pos_data);
    }
    else if (!strcmp(argv[i],"--neg"))
    {
      char* rosbag_file = argv[++i]; 
      char* scan_topic = argv[++i];
      tld.loadNegData(rosbag_file, scan_topic, train_neg_data);
    }
    else if (!strcmp(argv[i],"--test_pos"))
    {
      char* rosbag_file = argv[++i]; 
      char* scan_topic = argv[++i];
      tld.loadPosData(rosbag_file, scan_topic, test_pos_data);
    }
    else if (!strcmp(argv[i],"--test_neg"))
    {
      char* rosbag_file = argv[++i]; 
      char* scan_topic = argv[++i];
      tld.loadNegData(rosbag_file, scan_topic, test_neg_data);
    }
  }

  // Error check the loaded data
  if (train_pos_data.empty() or train_neg_data.empty()) //or tld.test_data_.size() == 0)
  {
    ROS_ERROR("data not loaded from rosbags properly \n");
  }
  else
  {
    printf("\n  Total positive training samples: %i \t Total negative training samples: %i \n", (int)train_pos_data.size(), (int)train_neg_data.size());
    printf("  Total positive test samples: %i \t Total negative test samples: %i \n\n", (int)test_pos_data.size(), (int)test_neg_data.size());
  }

  // Check if we want to do feature selection
  bool feature_selection;
  nh.param("feature_selection", feature_selection, false);
  if (feature_selection)
  {
    // Do a greedy forward search of features to choose best set
    // We'll also need the number of cross validation folds
    int cross_val_folds;
    nh.param("cross_validatation_folds", cross_val_folds, 5);       
    int best_num_correct_so_far = 0;    
    vector<int> features_included_so_far;
    bool improvement_found = true;    
    while (improvement_found == true)
    {
      improvement_found = false;

      vector<int> total_pos_vector; // vector storing the results of each newly included feature
      // Iterate once over each feature
      for (int i = 0; i < train_pos_data[0].size(); i++) 
      {
        // Check we haven't already included this feature
        bool feature_already_included = false;
        for (int j = 0; j < features_included_so_far.size(); j++)
          if (features_included_so_far[j] == i)
            feature_already_included = true;
        if (feature_already_included)
        {
          total_pos_vector.push_back(-1);
          continue;
        }
          
        // Populate our subset of data with that feature and <features_included_so_far>
        vector< vector<float> > train_pos_data_subset;
        vector< vector<float> > train_neg_data_subset;
        for (int j = 0; j < train_pos_data.size(); j++)
        {
          vector<float> sample_vector;
          for (int k = 0; k < features_included_so_far.size(); k++) // include features <from features_included_so_far>
            sample_vector.push_back(train_pos_data[j][k]); 
          sample_vector.push_back(train_pos_data[j][i]); // include the new feature
          train_pos_data_subset.push_back(sample_vector);
        }
        for (int j = 0; j < train_neg_data.size(); j++)
        {
          vector<float> sample_vector;
          for (int k = 0; k < features_included_so_far.size(); k++) // include features <from features_included_so_far>
            sample_vector.push_back(train_neg_data[j][k]); 
          sample_vector.push_back(train_neg_data[j][i]);  // include the new feature
          train_neg_data_subset.push_back(sample_vector);
        }
        // Test out the subset of data
        int correct_pos = 0; 
        int correct_neg = 0;
        int total_pos = 0;
        int total_neg = 0;
        tld.crossValidate(train_pos_data_subset, train_neg_data_subset, cross_val_folds, correct_pos, correct_neg, total_pos, total_neg);
        total_pos_vector.push_back(correct_pos + correct_neg);
      }   

      // Find out which newly included feature gave us the best improvement and include it in <features_included_so_far>
      int best_feat = -1;
      for (int i = 0; i < total_pos_vector.size(); i++)
      {
        printf("Feature:%i correct:%i \n", i, total_pos_vector[i]);
        if (total_pos_vector[i] > best_num_correct_so_far)
        {
          best_feat = i;
          best_num_correct_so_far = total_pos_vector[i];
          improvement_found = true;          
        }
      }
      features_included_so_far.push_back(best_feat);     

      printf("Included feature:%i \n\n", features_included_so_far.back());
    }

    printf("\nBest features are: ");  
    for (int j = 0; j < features_included_so_far.size()-1; j++)
    {  
      printf("%i, ", features_included_so_far[j]);
    }
    printf("\n");
    printf("To use these features for training, you should comment out all others in calc_cluster_features.cpp \n");
    return 0;
  }
  else // We're not doing feature selection
  {
    // Check if we want to do cross validation
    bool cross_validate;
    nh.param("cross_validate", cross_validate, false);
    // Get the number of folds for cross validation
    int cross_val_folds;
    nh.param("cross_validatation_folds", cross_val_folds, 5);    
    if (cross_validate)
    {
      printf("Cross validating...");
      int correct_pos = 0; // for recording cross validation scores
      int correct_neg = 0;
      int total_pos = 0;
      int total_neg = 0;
      tld.crossValidate(train_pos_data, train_neg_data, cross_val_folds, correct_pos, correct_neg, total_pos, total_neg);
      // Print cross validation results
      printf (" results: \n");
      printf("  Positive: %i/%i \t\t Error: %g%%\n", correct_pos, total_pos, 100.0 - 100.0*(float)(correct_pos)/total_pos);
      printf("  Negative: %i/%i \t\t Error: %g%%\n", correct_neg, total_neg, 100.0 - 100.0*(float)(correct_neg)/total_neg);
      printf("  Combined: %i/%i \t\t Error: %g%%\n\n", correct_pos+correct_neg, total_pos+total_neg, 100.0 - 100.0*(float)(correct_pos+correct_neg)/(total_pos+total_neg));
    }

    // Regular training using all training data
    printf("Training classifier...");
    tld.train(train_pos_data, train_neg_data);
    printf("done! \n\n");

    // Test classifier
    printf("Testing classifier...\n");
    // Test on training set
    printf(" training set: \n");
    int correct_pos = 0;
    int correct_neg = 0;
    tld.test(train_pos_data, train_neg_data, correct_pos, correct_neg);  
    printf("   Positive: %d/%d \t\t Error: %g%%\n", correct_pos, (int)train_pos_data.size(), 100.0 - 100.0*(float)(correct_pos)/(int)train_pos_data.size());
    printf("   Negative: %d/%d \t\t Error: %g%%\n", correct_neg, (int)train_neg_data.size(), 100.0 - 100.0*(float)(correct_neg)/(int)train_neg_data.size());
    printf("   Combined: %d/%d \t\t Error: %g%%\n\n", correct_pos + correct_neg, (int)train_pos_data.size() + (int)train_neg_data.size(), 100.0 - 100.0*(float)(correct_pos + correct_neg)/((int)train_pos_data.size() + (int)train_neg_data.size()));
    // Test on test set
    printf(" test set: \n");
    correct_pos = 0;
    correct_neg = 0;
    tld.test(test_pos_data, test_neg_data, correct_pos, correct_neg);  
    printf("   Positive: %d/%d \t\t Error: %g%%\n", correct_pos, (int)test_pos_data.size(), 100.0 - 100.0*(float)(correct_pos)/(int)test_pos_data.size());
    printf("   Negative: %d/%d \t\t Error: %g%%\n", correct_neg, (int)test_neg_data.size(), 100.0 - 100.0*(float)(correct_neg)/(int)test_neg_data.size());
    printf("   Combined: %d/%d \t\t Error: %g%%\n\n", correct_pos + correct_neg, (int)test_pos_data.size() + (int)test_neg_data.size(), 100.0 - 100.0*(float)(correct_pos + correct_neg)/((int)test_pos_data.size() + (int)test_neg_data.size()));

    // Saving
    printf("Saving classifier as: %s\n", tld.save_file_.c_str());    
    tld.save();

    return 0;

  }
}
