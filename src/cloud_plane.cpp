/*
adapted from 
PCL-ROS-cluster-segmentation
Author: Sean Cassero
*/

#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <shape_msgs/Plane.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <obj_segmentation/SegmentedClustersArray.h>

class segmentation {
    private:
        ros::NodeHandle m_nh;
        ros::Subscriber m_sub;
		ros::Publisher m_pub_pose;
        ros::Publisher m_pub_shape;
        ros::Publisher m_pub_objects;
        double minX = -10.0; 
        double maxX = 10.0; 
        double minY = -0.5; 
        double maxY = 0.2;
        double minZ = 0.3; 
        double maxZ = 2.0;
        double distance_threshold = 0.01; //1cm
        double cluster_tolerance = 0.02; //2cm
        int min_cluster_size = 1500;
        int max_cluster_size = 25000;
        double leaf_size = 0.01;
        std::string  input_cloud = "/points";
        std::string  output_cloud = "/points";
        void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    public:
        explicit segmentation(ros::NodeHandle nh) : m_nh(nh)  {
            ros::param::get("~input_cloud", input_cloud);
            ros::param::get("~output_cloud", output_cloud);
            ros::param::get("~minX", minX);
            ros::param::get("~maxX", maxX);
            ros::param::get("~minY", minY);
            ros::param::get("~maxY", maxY);
            ros::param::get("~minZ", minZ);
            ros::param::get("~maxZ", maxZ);
            ros::param::get("~distance_threshold", distance_threshold);
            ros::param::get("~cluster_tolerance", cluster_tolerance);
            ros::param::get("~min_cluster_size", min_cluster_size);
            ros::param::get("~max_cluster_size", max_cluster_size);
            ros::param::get("~leaf_size", leaf_size);

            // define the subscriber and publisher
            m_sub = m_nh.subscribe (input_cloud.c_str(), 1, &segmentation::cloud_cb, this);
            m_pub_pose = m_nh.advertise<geometry_msgs::PoseStamped> ("/object_pose",1);
            m_pub_shape = m_nh.advertise<shape_msgs::Plane> ("/object_plane",1);
            m_pub_objects = m_nh.advertise<sensor_msgs::PointCloud2> ("/object",1);
            ROS_INFO("\n leaf_size: %f\n distance_threshold: %f\n cluster_tolerance: %f\n min_cluster_size: %d\n max_cluster_size: %d\n input_cloud: %s\n", 
                        leaf_size, distance_threshold, cluster_tolerance, min_cluster_size, max_cluster_size, input_cloud.c_str());
            ROS_INFO("\n minX: %f\n maxX: %f\n minY: %f\n maxY: %f\n minZ: %f\n maxZ: %f\n", 
                        minX, maxX, minY, maxY, minZ, maxZ);
      }

}; // end class definition


// define callback function
void segmentation::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Convert to PCL data type
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform voxel grid downsampling filtering
  pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (leaf_size, leaf_size, leaf_size);
  sor.filter (*cloudFilteredPtr);

  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr (xyz_cloud); // need a boost shared pointer for pcl function inputs

  // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
  pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);

  //perform passthrough filtering to remove table leg
  // create a pcl object to hold the passthrough filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);

  // Create the filtering object
  pcl::CropBox<pcl::PointXYZRGB> pass;
  pass.setInputCloud (xyzCloudPtr);
  pass.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
  pass.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
  //pass.setFilterFieldName ("y");
  //pass.setFilterLimits (min_range, max_range);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*xyzCloudPtrFiltered);

  // create a pcl object to hold the ransac filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered (xyz_cloud_ransac_filtered);

  pcl::PointCloud<pcl::PointXYZRGB> *xyz_table_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzTableCloudPtrRansacFiltered (xyz_table_cloud_ransac_filtered);

  // perform ransac planar filtration to remove table top
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
  // Optional
  seg1.setOptimizeCoefficients (true);
  // Mandatory
  seg1.setModelType (pcl::SACMODEL_PLANE);
  seg1.setMethodType (pcl::SAC_RANSAC);
  seg1.setDistanceThreshold (distance_threshold);
  seg1.setInputCloud (xyzCloudPtrFiltered);
  seg1.segment (*inliers, *coefficients);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*xyzCloudPtrRansacFiltered);

  sensor_msgs::PointCloud2 objects_output;
  pcl::PCLPointCloud2 objects_outputPCL;
  // convert to pcl::PCLPointCloud2
  pcl::toPCLPointCloud2( *xyzCloudPtrRansacFiltered, objects_outputPCL);
  // Convert to ROS data type
  pcl_conversions::fromPCL(objects_outputPCL, objects_output);
  objects_output.header = cloud_msg->header;
  //publish the table
  m_pub_objects.publish(objects_output);


  int count = 0;
  float running_x = 0.0;
  float running_y = 0.0;
  float running_z = 0.0;
  float x,y,z;
/*
  for (int index=0; index < xyzCloudPtrRansacFiltered->points.size(); index++)  
  {
    //std::cout<<index<<std::endl;
    //std::cout<<xyzCloudPtrRansacFiltered->points[index]<<std::endl;
    //std::cout<<xyzCloudPtrRansacFiltered->points[index].x<<std::endl;
    //std::cout<<xyzCloudPtrRansacFiltered->points[index].y<<std::endl;
    //std::cout<<xyzCloudPtrRansacFiltered->points[index].z<<std::endl;
    x = xyzCloudPtrRansacFiltered->points[index].x;
    y = xyzCloudPtrRansacFiltered->points[index].y;
    z = xyzCloudPtrRansacFiltered->points[index].z;

    running_x += x;
    running_y += y;
    running_z += z;
    count++;

  }

  x = running_x/count;
  y = running_y/count;
  z = running_z/count;
*/
    x = xyzCloudPtrRansacFiltered->points[0].x;
    y = xyzCloudPtrRansacFiltered->points[0].y;
    z = xyzCloudPtrRansacFiltered->points[0].z;

  float a = coefficients->values[0];
  float b = coefficients->values[1];
  float c = coefficients->values[2];
  float d = coefficients->values[3];

  ROS_INFO("a:%f b:%f c:%f d:%f", a,b,c,d);

  shape_msgs::Plane plane;
  //double coef[] = {a,b,c,d};
  plane.coef[0] = a;
  plane.coef[1] = b;
  plane.coef[2] = c;
  plane.coef[3] = d;
  m_pub_shape.publish(plane);

  float cos_theta_x = a/(sqrt(a*a + b*b + c*c));
  float cos_theta_y = b/(sqrt(a*a + b*b + c*c));
  float cos_theta_z = c/(sqrt(a*a + b*b + c*c));

  float roll = acos(cos_theta_x);
  float pitch = acos(cos_theta_y);
  float yaw = acos(cos_theta_z);

  ROS_INFO("x:%f y:%f z:%f", x,y,z);
  ROS_INFO("roll:%f pitch:%f yaw:%f", roll*(180/3.14), pitch*(180/3.14), yaw*(180/3.14));

  ros::Time now = ros::Time::now();
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = cloud_msg->header;
  pose_stamped.header.stamp = now;

  pose_stamped.pose.position.z = 0.50;

  float qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
  float qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
  float qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
  float qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);

  ROS_INFO("qx:%f qy:%f qz:%f qw:%f", qx, qy, qz, qw);
  ROS_INFO("============================================");
  pose_stamped.pose.position.x = x;
  pose_stamped.pose.position.y = y;
  pose_stamped.pose.position.z = z;

  pose_stamped.pose.orientation.x = qx;
  pose_stamped.pose.orientation.y = qy;
  pose_stamped.pose.orientation.z = qz;
  pose_stamped.pose.orientation.w = qw;
  m_pub_pose.publish(pose_stamped);


}



int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "segmentation");
  ros::NodeHandle nh;
  segmentation segs(nh);
  while(ros::ok())
    ros::spin ();
}

