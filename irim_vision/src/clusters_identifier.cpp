/* CLUSTERS IDENTIFIER - Classifies the clusters published by segmentation
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

// ROS Includes
#include <ros/ros.h>


// PCL Includes

// Custom Includes
#include <irim_vision/SegmentedClustersArray.h>
#include <irim_vision/IdentifiedClustersArray.h>

// Defines
#define     DEBUG   1       // Prints out additional stuff


// Class definition
class ClustersIdentifier {

    public:

    // Class constructor
    explicit ClustersIdentifier (ros::NodeHandle nh) : i_nh(nh) {

        // Build the subscriber and publisher
         i_sub = i_nh.subscribe ("irim_vision/clusters", 10, &ClustersIdentifier::cluster_cb, this);
         i_pub = i_nh.advertise<irim_vision::IdentifiedClustersArray> ("irim_vision/identified_clusters", 1);
    }

    private:
    
    // ROS stuff
    ros::NodeHandle i_nh;
    ros::Publisher i_pub;
    ros::Subscriber i_sub;

    // Callback for the segmented clusters array
    void cluster_cb(const irim_vision::SegmentedClustersArrayConstPtr& cloud_msg);

}; // End class definition

void ClustersIdentifier::cluster_cb(const irim_vision::SegmentedClustersArrayConstPtr& cloud_msg){

    

}

// MAIN

int main(int argc, char **argv)
{
  ros::init(argc, argv, "clusters_identifier");
  ros::NodeHandle nh;

  ClustersIdentifier clusters_identifier(nh);

  // Spin until ROS is shutdown
  while (ros::ok())
    ros::spin();

  return 0;
}