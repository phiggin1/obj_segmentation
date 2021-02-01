/*
adapted from 
PCL-ROS-cluster-segmentation
Author: Sean Cassero
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/Detection3DArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <husky_perception/SegmentedClustersArray.h>

class segmentation {
	public:
		explicit segmentation(ros::NodeHandle nh) : m_nh(nh)  {

      ros::param::get("~input_cloud", input_cloud);
			ros::param::get("~max_range", max_range);
			ros::param::get("~min_range", min_range);
			ros::param::get("~distance_threshold", distance_threshold);
			ros::param::get("~cluster_tolerance", cluster_tolerance);
			ros::param::get("~min_cluster_size", min_cluster_size);
			ros::param::get("~max_cluster_size", max_cluster_size);

      // define the subscriber and publisher
			m_sub = m_nh.subscribe ("/camera/depth/points", 1, &segmentation::cloud_cb, this);
			m_pub_obj = m_nh.advertise<sensor_msgs::PointCloud2> ("/objects",1);
			m_pub_tbl = m_nh.advertise<sensor_msgs::PointCloud2> ("/table",1);
			m_clusterPub = m_nh.advertise<husky_perception::SegmentedClustersArray> ("object_clusters",1);

			ROS_INFO("\n min_range: %f\n max_range: %f\n distance_threshold: %f\n cluster_tolerance: %f\n min_cluster_size: %d\n max_cluster_size: %d\n input_cloud: %s\n", 
        min_range, max_range, distance_threshold, cluster_tolerance, min_cluster_size, max_cluster_size, input_cloud.c_str());
	  }

	private:
		ros::NodeHandle m_nh;
		ros::Publisher m_pub_obj;
		ros::Publisher m_pub_tbl;
		ros::Subscriber m_sub;
		ros::Publisher m_clusterPub;
		double max_range = 1.5; //1.5 m
		double min_range = 0.5; //0.5 m
		double distance_threshold = 0.01; //1cm
		double cluster_tolerance = 0.02; //2cm
		int min_cluster_size = 1500;
		int max_cluster_size = 25000;
    std::string  input_cloud = "/camera/depth/points";
		void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

}; // end class definition


// define callback function
void segmentation::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // declare an instance of the SegmentedClustersArray message
  husky_perception::SegmentedClustersArray CloudClusters;
  CloudClusters.header = cloud_msg->header;

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
  sor.setLeafSize (0.01, 0.01, 0.01);
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
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (xyzCloudPtr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (min_range, max_range);
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

  //extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*xyzCloudPtrRansacFiltered);

  //get hhe points for the tabel
  extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*xyzTableCloudPtrRansacFiltered);
  sensor_msgs::PointCloud2 tbl_output;
  pcl::PCLPointCloud2 tbl_outputPCL;
  // convert to pcl::PCLPointCloud2
  pcl::toPCLPointCloud2( *xyzTableCloudPtrRansacFiltered, tbl_outputPCL);
  // Convert to ROS data type
  pcl_conversions::fromPCL(tbl_outputPCL, tbl_output);
  tbl_output.header = cloud_msg->header;
  CloudClusters.surfaces.push_back(tbl_output);
  //publish the table
  m_pub_tbl.publish(tbl_output);

  //get the points for the objects
  // declare the output variable instances
  sensor_msgs::PointCloud2 out;
  pcl::PCLPointCloud2 outPCL;
  // convert to pcl::PCLPointCloud2
  pcl::toPCLPointCloud2( *xyzCloudPtrRansacFiltered ,outPCL);
  // Convert to ROS data type
  pcl_conversions::fromPCL(outPCL, out);
  //publish the objects
  //m_pub_obj.publish(out);

  // perform euclidean cluster segmentation to seporate individual objects
  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (xyzCloudPtrRansacFiltered);

  // create the extraction object for the clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // specify euclidean cluster parameters
  ec.setClusterTolerance (cluster_tolerance); // 2cm
  ec.setMinClusterSize (min_cluster_size);
  ec.setMaxClusterSize (max_cluster_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud (xyzCloudPtrRansacFiltered);
  // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
  ec.extract (cluster_indices);

  sensor_msgs::PointCloud2 output;
  pcl::PCLPointCloud2 outputPCL;

  // here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them seporately
  int i = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
    // create a pcl object to hold the extracted cluster
    pcl::PointCloud<pcl::PointXYZRGB> *cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (cluster);

    // now we are in a vector of indices pertaining to a single cluster.
    // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
        clusterPtr->points.push_back(xyzCloudPtrRansacFiltered->points[*pit]);
    }

    // convert to pcl::PCLPointCloud2
    pcl::toPCLPointCloud2( *clusterPtr ,outputPCL);
    // Convert to ROS data type
    pcl_conversions::fromPCL(outputPCL, output);

    output.header = cloud_msg->header;

    m_pub_obj.publish(output);

    // add the cluster to the array message
    CloudClusters.clusters.push_back(output);
    i++;
  }
  ROS_INFO("#clusters: %d\n", i);
  m_clusterPub.publish(CloudClusters);

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

