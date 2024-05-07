/*
adapted from 
PCL-ROS-cluster-segmentation
Author: Sean Cassero
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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

class segmentation {
	private:
		ros::NodeHandle m_nh;
		ros::Publisher m_pub_filtered;
		ros::Subscriber m_sub;
		double minX = -10.0; 
		double maxX = 10.0; 
		double minY = -10.0; 
		double maxY = 10.0;
		double minZ = 0.1; 
		double maxZ = 2.0;
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
			ros::param::get("~leaf_size", leaf_size);

      // define the subscriber and publisher
			m_sub = m_nh.subscribe (input_cloud.c_str(), 1, &segmentation::cloud_cb, this);
			m_pub_filtered = m_nh.advertise<sensor_msgs::PointCloud2> (output_cloud,1);

			ROS_INFO("\n leaf_size: %f\n input_cloud: %s\n", 
        				leaf_size, input_cloud.c_str());
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
  pcl::	CropBox<pcl::PointXYZRGB> pass;
  pass.setInputCloud (xyzCloudPtr);
  pass.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
  pass.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
  //pass.setFilterFieldName ("y");
  //pass.setFilterLimits (min_range, max_range);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*xyzCloudPtrFiltered);

  ros::Time now = (ros::Time::now());//-(ros::Duration(0, 500000000));
  //ROS_INFO("stamp: %d", now);
  now = now-(ros::Duration(0, 500000000));
  //ROS_INFO("stamp: %d", now);


  pcl::PCLPointCloud2 filtered_PCL;
  sensor_msgs::PointCloud2 filtered_PCL_output;
  // convert to pcl::PCLPointCloud2
  pcl::toPCLPointCloud2( *xyzCloudPtrFiltered ,filtered_PCL);
  // Convert to ROS data type

  pcl_conversions::fromPCL(filtered_PCL, filtered_PCL_output);
  filtered_PCL_output.header = cloud_msg->header;
  filtered_PCL_output.header.stamp = now;
  m_pub_filtered.publish(filtered_PCL_output);

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

