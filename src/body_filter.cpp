#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include "tf/transform_listener.h"
#include "filters/filter_chain.h"

class GenericLaserScanFilterNode
{
protected:
  // Our NodeHandle
  ros::NodeHandle nh_;

  // Components for tf::MessageFilter
  ros::Subscriber m_sub;

  // Filter Chain
  filters::FilterChain<sensor_msgs::PointCloud2> filter_chain_;

  // Components for publishing
  sensor_msgs::PointCloud2 msg_;
  ros::Publisher output_pub_;

public:
  // Constructor
  GenericLaserScanFilterNode() :
      filter_chain_("sensor_msgs::PointCloud2")
  {
    m_sub = nh_.subscribe ("/camera/depth/points", 1, &GenericLaserScanFilterNode::callback, this);

    // Configure filter chain
    //filter_chain_.configure("~");
    filter_chain_.configure("cloud_filter_chain");

    // Advertise output
    output_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output", 1000);
  }

  // Callback
  void callback(const sensor_msgs::PointCloud2::ConstPtr& msg_in)
  {
    // Run the filter chain
    filter_chain_.update (*msg_in, msg_);

    // Publish the output
    output_pub_.publish(msg_);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_filter_node");

  GenericLaserScanFilterNode t;
  ros::spin();

  return 0;
}
