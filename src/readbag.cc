#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <readbag/tqdm.h>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <filesystem>

void CreateDirectory(std::string dir_name) {
  std::filesystem::path dir(dir_name);
  if (std::filesystem::create_directory(dir)) {
    std::cout << "Directory Created: " << dir_name << std::endl;
  } else {
    if (std::filesystem::exists(dir)) {
      std::cout << "Directory already exists: " << dir_name << std::endl;
    } else {
      std::cout << "Failed to create directory: " << dir_name << std::endl;
    }
  }
}

int main(int argc, char **argv) {
  if (argc < 2) {
    RCLCPP_ERROR(rclcpp::get_logger("readbag"),
                 "Usage: ros2 run readbag readbag <bag_file>");
    RCLCPP_ERROR(
        rclcpp::get_logger("readbag"),
        "Note that the output folders will be saved in the working directory.");
    return 1;
  }

  CreateDirectory("pcd");
  CreateDirectory("image");

  rosbag2_cpp::Reader reader;
  reader.open(argv[1]);
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_ser;
  rclcpp::Serialization<sensor_msgs::msg::CompressedImage> img_ser;

  int clouds = 0, images = 0;
  for (auto topic : reader.get_metadata().topics_with_message_count) {
    if (topic.topic_metadata.name == "/aeva/AEVA/point_cloud_compensated") {
      clouds = topic.message_count;
    } else if (topic.topic_metadata.name == "/image_raw/compressed") {
      images = topic.message_count;
    }
  }
  int total = clouds + images;
  std::cout << "Point cloud messages: " << clouds << std::endl;
  std::cout << "Image messages: " << images << std::endl;
  std::cout << "Total messages: " << total << std::endl;
  int progress = 0;

  tqdm bar;
  while (reader.has_next()) {
    rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();
    if (msg->topic_name == "/aeva/AEVA/point_cloud_compensated") {
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      pc_ser.deserialize_message(&serialized_msg, cloud_msg.get());
      pcl::PointCloud<pcl::PointXYZI> cloud;
      pcl::fromROSMsg(*cloud_msg, cloud);
      pcl::io::savePCDFileASCII(
          "pcd/" + std::to_string(msg.get()->time_stamp) + ".pcd", cloud);
      ++progress;
      bar.progress(progress, total);
    } else if (msg->topic_name == "/image_raw/compressed") {
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      auto img_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
      img_ser.deserialize_message(&serialized_msg, img_msg.get());
      cv::Mat image = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
      cv::imwrite("image/" + std::to_string(msg.get()->time_stamp) + ".png",
                  image);
      ++progress;
      bar.progress(progress, total);
    }
  }
  reader.close();
}
