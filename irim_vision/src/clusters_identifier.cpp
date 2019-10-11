/* CLUSTERS IDENTIFIER - Classifies the clusters published by segmentation
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

// ROS Includes
#include <ros/ros.h>

// Other Includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>

// Custom Includes
#include <irim_vision/SegmentedClustersArray.h>
#include <irim_vision/IdentifiedCluster.h>
#include <irim_vision/IdentifiedClustersArray.h>

// Defines
#define     DEBUG   1       // Prints out additional stuff


// Defining colors struct and some basic colors
struct Color {

    // Constructora
    Color() {
        this->name = "black";
        this->r = 0; this->g = 0; this->b = 0;
    }

    Color(std::string name_, int r_, int g_, int b_) {
        this->name = name_;
        this->r = r_; this->g = g_; this->b = b_;
    }

    // Variables
    std::string name;
    int r, g, b;
};

Color red("red", 255, 0, 0);            // obj_id = 1
Color green("green", 0, 255, 0);        // obj_id = 2
Color blue("blue", 0, 0, 255);          // obj_id = 3
Color black("black", 0, 0, 0);          // obj_id = 4
Color white("red", 255, 255, 255);      // obj_id = 5


// Class definition
class ClustersIdentifier {

    public:

    // Class constructor
    explicit ClustersIdentifier (ros::NodeHandle nh) : i_nh(nh) {

        // Build the subscriber and publisher
        this->i_sub = i_nh.subscribe ("irim_vision/clusters", 10, &ClustersIdentifier::cluster_cb, this);
        this->i_pub = i_nh.advertise<irim_vision::IdentifiedClustersArray> ("irim_vision/identified_clusters", 1);

        // Build the colors and pushback
        this->known_colors = {red, green, blue, black, white};

    }

    // Class destructor
    ~ClustersIdentifier () {
        // Nothig to do here
    }

    private:
    
    // ROS stuff
    ros::NodeHandle i_nh;
    ros::Publisher i_pub;
    ros::Subscriber i_sub;

    // Known colors vector
    std::vector<Color> known_colors;

    // PCL Elements
    pcl::PCLPointCloud2 tmp_cluster;
    pcl::PointCloud<pcl::PointXYZRGB> cluster_xyzrgb;

    // Other vars
    irim_vision::SegmentedClustersArray saved_msg;

    // Callback for the segmented clusters array
    void cluster_cb(const irim_vision::SegmentedClustersArrayConstPtr& cloud_msg);

    // Function for assigning id according to color and poses through centroid computation
    int assign_pose_id(sensor_msgs::PointCloud2 cluster_in, geometry_msgs::Pose& assigned_pose);

}; // End class definition

void ClustersIdentifier::cluster_cb(const irim_vision::SegmentedClustersArrayConstPtr& seg_clusters_msg) {

    // Saving the message, creating the output identified clusters array and tmp variables
    this->saved_msg = *seg_clusters_msg;
    std_msgs::Header tmp_header;
    irim_vision::IdentifiedCluster tmp_ident_cluster;
    irim_vision::IdentifiedClustersArray ouput_msg;

    // Processing single clusters and saving if inside square
    for (int i = 0; i < this->saved_msg.clusters.size(); i++) {

        geometry_msgs::Pose tmp_pose;

        // Assigning the id
        tmp_ident_cluster.obj_id = assign_pose_id(this->saved_msg.clusters.at(i), tmp_pose);

        // Assigning pose
        tmp_ident_cluster.pose = tmp_pose;

        // Assigning header
        tmp_header.stamp = ros::Time::now();
        tmp_ident_cluster.header = tmp_header;

        // Pushing back
        ouput_msg.ident_clusters.push_back(tmp_ident_cluster);

    }

    // Publishing the identified clusters array
    this->i_pub.publish(ouput_msg);

}

// Function for assigning id according to color
int ClustersIdentifier::assign_pose_id(sensor_msgs::PointCloud2 cluster_in, geometry_msgs::Pose& assigned_pose) {

    int closest_id = 1;
    double closest_dist = 3 * std::pow(255, 2);

    // Converting to PCL format
    pcl_conversions::toPCL(cluster_in, this->tmp_cluster);
    pcl::fromPCLPointCloud2(this->tmp_cluster, this->cluster_xyzrgb);

    // Computing the average color of the cluster
    pcl::CentroidPoint<pcl::PointXYZRGB> centroid_finder;
    pcl::PointXYZRGB curr_centroid;
    for (auto it : cluster_xyzrgb.points) {
        centroid_finder.add(it);
        if (DEBUG) ROS_INFO_STREAM("The current point of the cluster has col: (" << it.r << ", " << it.g << ", " << it.b << ")");
    }
    centroid_finder.get(curr_centroid);

    if (DEBUG) ROS_INFO_STREAM("The centroid of the cluster is pos: (" << curr_centroid.x << ", " << curr_centroid.y << ", " << 
        curr_centroid.z << ") / col: (" << curr_centroid.r << ", " << curr_centroid.g << ", " << curr_centroid.b << ")");

    // // Iterating the colors and checking the nearest
    // for (int i = 0; i < this->known_colors.size(); i++) {

    //     int curr_dist = 
    //     if (this->)

    // }

}

// MAIN

int main(int argc, char **argv)
{
  ros::init(argc, argv, "clusters_identifier");
  ros::NodeHandle nh;

  ClustersIdentifier clusters_identifier(nh);

  // Spin until ROS is shutdown
  while (ros::ok()){
      ros::spin();
  }

  return 0;
}