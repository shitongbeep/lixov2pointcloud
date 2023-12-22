#include <cassert>
#include <iostream>
#include <string>

#include "livox_convert/CustomMsg.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "convert");
  ros::NodeHandle nh;

  std::string input_bag_filename, output_bag_filename;
  std::string imu_topic, lidar_topic;
  nh.param<std::string>("/convert_node/input_bag", input_bag_filename, "");
  nh.param<std::string>("/convert_node/output_bag", output_bag_filename, "");
  nh.param<std::string>("/convert_node/imu_topic", imu_topic, "");
  nh.param<std::string>("/convert_node/lidar_topic", lidar_topic, "");

  std::cout << "lidar_topic: " << lidar_topic << std::endl;
  std::cout << "imu_topic: " << imu_topic << std::endl;
  std::cout << "input: " << input_bag_filename << std::endl;
  std::cout << "output: " << output_bag_filename << std::endl;
  assert(!input_bag_filename.empty() && !output_bag_filename.empty() &&
         !imu_topic.empty() && !lidar_topic.empty());

  rosbag::Bag input_bag(input_bag_filename, rosbag::bagmode::Read);
  rosbag::Bag output_bag(output_bag_filename, rosbag::bagmode::Write);

  rosbag::View imu_view(input_bag, rosbag::TopicQuery(imu_topic));
  rosbag::View pointcloud_view(input_bag, rosbag::TopicQuery(lidar_topic));

  for (const rosbag::MessageInstance& m : imu_view) {
    sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
    if (imu_msg != nullptr) {
      output_bag.write(imu_topic, imu_msg->header.stamp, imu_msg);
    }
    std::cout << "convert imu at " << std::fixed
              << imu_msg->header.stamp.toSec() << std::endl;
  }

  sensor_msgs::PointCloud::Ptr pc_msg = nullptr;
  sensor_msgs::PointCloud2::Ptr pc2_msg = nullptr;
  for (const rosbag::MessageInstance& m : pointcloud_view) {
    livox_convert::CustomMsg::ConstPtr livox_msg = m.instantiate<livox_convert::CustomMsg>();
    if (livox_msg != nullptr) {
      pc_msg.reset(new sensor_msgs::PointCloud);
      pc2_msg.reset(new sensor_msgs::PointCloud2);
      pc_msg->header = livox_msg->header;
      pc_msg->points.reserve(livox_msg->point_num);
      pc_msg->channels.resize(2);
      pc_msg->channels.at(0).name = "intensity";
      pc_msg->channels.at(0).values.reserve(livox_msg->point_num);  // intensity
      pc_msg->channels.at(1).name = "time";
      pc_msg->channels.at(1).values.reserve(livox_msg->point_num);  // time
      for (const auto& livox_pt : livox_msg->points) {
        geometry_msgs::Point32 geo_pt;
        geo_pt.x = livox_pt.x;
        geo_pt.y = livox_pt.y;
        geo_pt.z = livox_pt.z;
        pc_msg->points.emplace_back(geo_pt);
        pc_msg->channels.at(0).values.emplace_back(livox_pt.reflectivity);
        pc_msg->channels.at(1).values.emplace_back(livox_pt.offset_time);
      }
      convertPointCloudToPointCloud2(*pc_msg, *pc2_msg);
      pc2_msg->header = pc_msg->header;
      output_bag.write(lidar_topic, pc_msg->header.stamp, pc2_msg);
    }
    std::cout << "convert lidar at " << std::fixed
              << pc_msg->header.stamp.toSec() << std::endl;
  }

  input_bag.close();
  output_bag.close();
  std::cout << "finish convert" << std::endl;
  return 0;
}
