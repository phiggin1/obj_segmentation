#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "filters/filter_chain.h"
#include <ros/console.h>

class robot_body_filter
{      
  private:
    filters::FilterChain<sensor_msgs::PointCloud2> filter_chain_;   
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
  	std::string  output_cloud = "/points_filtered";
		std::string  input_cloud = "/camera/depth/points";


  public:
    robot_body_filter() : filter_chain_("sensor_msgs::PointCloud2")
    {
      ros::param::get("~input_cloud", input_cloud);
      ros::param::get("~output_cloud", output_cloud);
      filter_chain_.configure("cloud_filter_chain");

      pub_ = n_.advertise<sensor_msgs::PointCloud2>(output_cloud.c_str(), 1);
      sub_ = n_.subscribe(input_cloud.c_str(), 1, &robot_body_filter::callback, this);
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& input)
    {    
      sensor_msgs::PointCloud2 output;
      filter_chain_.update(*input, output);
      pub_.publish(output);
    }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "load_filter_chain");
  robot_body_filter rbf;
  ros::spin();
  return 0;
}