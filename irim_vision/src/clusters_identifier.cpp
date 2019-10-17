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
#include <pcl/point_types_conversion.h>

// Custom Includes
#include <irim_vision/SegmentedClustersArray.h>
#include <irim_vision/IdentifiedCluster.h>
#include <irim_vision/IdentifiedClustersArray.h>

// Defines
#define     DEBUG   1       // Prints out additional stuff

/* 
    BEWARE: the ws rectangle, colors and the basic pose orientation are hard coded
    TODO: Parse these values! 
*/

// Defining colors struct and some basic colors
struct Color {

    // Constructor
    Color() {
        this->name = "red"; this->id = 1;
        this->h_low = 0; this->h_high = 60;
        this->s_low = 0.25; this->s_high = 1.00;
        this->v_low = 0.25; this->v_high = 1.00;
    }

    Color(std::string name_, int id_, int h_low_, int h_high_, float s_low_, float s_high_, float v_low_, float v_high_) {
        this->name = name_; this->id = id_;
        this->h_low = h_low_; this->h_high = h_high_;
        this->s_low = s_low_; this->s_high = s_high_;
        this->v_low = v_low_; this->v_high = v_high_;
    }

    // Variables
    std::string name;
    int h_low, h_high;
    float s_low, s_high;
    float v_low, v_high;
    int id;
};

Color red("red", 1, 0, 60, 0.25, 1.00, 0.25, 1.00);                     // obj_id = 1
Color green("green", 2, 121, 180, 0.25, 1.00, 0.25, 1.00);              // obj_id = 2
Color blue("blue", 3, 200, 280, 0.00, 0.45, 0.45, 1.00);                // obj_id = 3
Color black("black", 4, 0, 360, 0.0, 0.45, 0.0, 0.55);                  // obj_id = 4
Color white("white", 5, 0, 360, 0.0, 0.35, 0.55, 1.00);                 // obj_id = 5
Color red2("red2", 1, 330, 360, 0.25, 1.00, 0.25, 1.00);                // obj_id = 1

Color other("other", 6, 0, 0, 0, 0, 0, 0);                              // obj_id = 6 (NOT IN VECTOR known_colors)


// Class definition
class ClustersIdentifier {

    public:

    // Class constructor
    explicit ClustersIdentifier (ros::NodeHandle nh) : i_nh(nh) {

        // Build the subscriber and publisher
        this->i_sub = i_nh.subscribe ("irim_vision/clusters", 10, &ClustersIdentifier::cluster_cb, this);
        this->i_pub = i_nh.advertise<irim_vision::IdentifiedClustersArray> ("irim_vision/identified_clusters", 1);

        // Build the colors and pushback
        this->known_colors = {red, green, blue, black, white, red2};

        // Assign the basic orientation
        this->basic_quat.x = 0.0; this->basic_quat.y = 0.0;
        this->basic_quat.z = 0.0; this->basic_quat.w = 1.0;

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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_xyzrgb_ptr;

    // Other vars
    irim_vision::SegmentedClustersArray saved_msg;

    // RECTAGLE OF WORKSPACE (only clusters inside this rectangle are considered)
    // TODO: Parse these values
    const double x_lo = 0.20; const double x_up = 0.95;
    const double y_l = 0.10; const double y_r = -0.40;

    // Basic orientation
    geometry_msgs::Quaternion basic_quat;

    // Callback for the segmented clusters array
    void cluster_cb(const irim_vision::SegmentedClustersArrayConstPtr& cloud_msg);

    // Function for assigning id according to color and poses through centroid computation
    bool assign_pose_id(sensor_msgs::PointCloud2 cluster_in, geometry_msgs::Pose& assigned_pose, int& assigned_id);

    // Aux function to unpack rgb of PointXYZRGB
    inline void unpack_rgb(pcl::PointXYZRGB point_in, int& r_out, int& g_out, int& b_out) {
        uint32_t rgb = *reinterpret_cast<int*>(&point_in.rgb);
        uint8_t r = (rgb >> 16) & 0x0000ff;
        uint8_t g = (rgb >> 8)  & 0x0000ff;
        uint8_t b = (rgb)       & 0x0000ff;
        r_out = r; g_out = g; b_out = b;
    }

    // Aux function for checking if point inside workspace rectangle
    inline bool is_inside(pcl::PointXYZRGB point_in) {
        if (point_in.x > x_lo && point_in.x < x_up) {
            if (point_in.y > y_r && point_in.y < y_l) {
                return true;
            }
        }
        return false;
    }

    // Aux function to find if hsv point in color range
    inline bool is_color(Color color, pcl::PointXYZHSV point) {
        if (point.h <= color.h_high && point.h >= color.h_low) {
            if (point.s <= color.s_high && point.s >= color.s_low) {
                if (point.v <= color.v_high && point.v >= color.v_low) {
                    return true;
                }
            }
        }
        return false;
    }

}; // End class definition

void ClustersIdentifier::cluster_cb(const irim_vision::SegmentedClustersArrayConstPtr& seg_clusters_msg) {

    // Saving the message, creating the output identified clusters array and tmp variables
    this->saved_msg = *seg_clusters_msg;
    std_msgs::Header tmp_header;
    irim_vision::IdentifiedCluster tmp_ident_cluster;
    irim_vision::IdentifiedClustersArray output_msg;

    // Processing single clusters and saving if inside square
    for (int i = 0; i < this->saved_msg.clusters.size(); i++) {

        geometry_msgs::Pose tmp_pose;
        int tmp_id;

        // Assigning the id and pose and checking if inside ws (if not go to next cluster)
        bool inside = assign_pose_id(this->saved_msg.clusters.at(i), tmp_pose, tmp_id);
        if (!inside) {
            ROS_WARN("The processed cluster is outside the ws!");
            continue;
        }

        // Assigning pose and id
        tmp_ident_cluster.pose = tmp_pose;
        tmp_ident_cluster.obj_id = tmp_id;

        // Assigning header
        tmp_header.stamp = ros::Time::now();
        tmp_ident_cluster.header = tmp_header;
        
        // Pushing back
        output_msg.ident_clusters.push_back(tmp_ident_cluster);

    }

    // Publishing the identified clusters always (because needed for checking grasp success)
    this->i_pub.publish(output_msg);

}

// Function for assigning id according to color
bool ClustersIdentifier::assign_pose_id(sensor_msgs::PointCloud2 cluster_in, geometry_msgs::Pose& assigned_pose, int& assigned_id) {

    int closest_id = 1;
    double closest_dist = 3 * std::pow(255, 2);

    // Converting to PCL format
    this->cluster_xyzrgb_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(cluster_in, *this->cluster_xyzrgb_ptr);

    // Computing the average color of the cluster
    pcl::CentroidPoint<pcl::PointXYZRGB> centroid_finder;
    pcl::PointXYZRGB curr_centroid;
    for (auto it : cluster_xyzrgb_ptr->points) {
        centroid_finder.add(it);
    }
    centroid_finder.get(curr_centroid);

    // Checking if centroid inside ws
    if (!this->is_inside(curr_centroid)) return false;

    // Unpacking and couting
    int r, g, b; 
    if (DEBUG) {
        this->unpack_rgb(curr_centroid, r, g, b);
        ROS_INFO_STREAM("The centroid of the cluster is pos: (" << curr_centroid.x << ", " << curr_centroid.y << ", " << 
            curr_centroid.z << ") / col: (" << r << ", " << g << ", " << b << ")");
    }

    // Converting to HSV
    pcl::PointXYZHSV hsv_point;
    pcl::PointXYZRGBtoXYZHSV(curr_centroid, hsv_point);

    if (DEBUG) {
        ROS_INFO_STREAM("The HSV of the cluster is hsv: (" << hsv_point.h << ", " << hsv_point.s << ", " << hsv_point.v << ")");
    }

    // Iterating the colors and checking the nearest and choosing id accordingly
    int chosen_id = 0;
    bool found_color = false;
    for (auto it : this->known_colors) {
        if (this->is_color(it, hsv_point)) {
            found_color = true;
            chosen_id = it.id;
            break;
        }
    }

    if (!found_color) {
        chosen_id = other.id;
    }

    if (DEBUG) {
        ROS_INFO_STREAM("The id (color) of the cluster is " << chosen_id << "!");
    }

    // Assigning the pose of the cluster
    geometry_msgs::Pose ass_pose;                   // Ass! Hihihihi XD
    ass_pose.position.x = curr_centroid.x;
    ass_pose.position.y = curr_centroid.y;
    ass_pose.position.z = curr_centroid.z;
    ass_pose.orientation = this->basic_quat;

    // Passing through ref and returning
    assigned_id = chosen_id;
    assigned_pose = ass_pose;
    return true;

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