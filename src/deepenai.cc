#include <Eigen/Dense>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using json = nlohmann::json;

std::vector<sensor_msgs::msg::CompressedImage> images;
sensor_msgs::msg::PointCloud2::SharedPtr cloud = nullptr;
std::vector<nav_msgs::msg::Odometry> odoms;

Eigen::Vector3d xyz(0.150, 0.190, -0.006);
Eigen::Vector3d rpy(-91.553, -0.683, -85.245);
Eigen::Affine3d Tlc =
    Eigen::Translation3d(xyz) *
    Eigen::AngleAxisd(rpy[2] * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy[1] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy[0] * M_PI / 180.0, Eigen::Vector3d::UnitX());

void CreateDirectory(std::string dir_name) {
  std::filesystem::path dir(dir_name);
  if (std::filesystem::create_directory(dir)) {
    std::cerr << "Directory Created: " << dir_name << std::endl;
  } else {
    if (std::filesystem::exists(dir)) {
      std::cerr << "Directory already exists: " << dir_name << std::endl;
    } else {
      std::cerr << "Failed to create directory: " << dir_name << std::endl;
    }
  }
}

std::optional<Eigen::Affine3d> GetPose(const builtin_interfaces::msg::Time &t) {
  auto p2 = std::lower_bound(odoms.begin(), odoms.end(), t,
                             [](const nav_msgs::msg::Odometry &odom,
                                const builtin_interfaces::msg::Time &t) {
                               return rclcpp::Time(odom.header.stamp) <
                                      rclcpp::Time(t);
                             });
  if (p2 != odoms.end() && p2 != odoms.begin()) {
    Eigen::Affine3d ret;
    auto p1 = std::prev(p2);
    double num =
        rclcpp::Time(t).seconds() - rclcpp::Time(p1->header.stamp).seconds();
    double den = rclcpp::Time(p2->header.stamp).seconds() -
                 rclcpp::Time(p1->header.stamp).seconds();
    double ration = num / den;
    tf2::Vector3 v1, v2;
    tf2::Quaternion q1, q2;
    tf2::fromMsg(p1->pose.pose.position, v1);
    tf2::fromMsg(p2->pose.pose.position, v2);
    tf2::fromMsg(p1->pose.pose.orientation, q1);
    tf2::fromMsg(p2->pose.pose.orientation, q2);
    tf2::Vector3 pos = v1.lerp(v2, ration);
    ret.translation().x() = pos.x();
    ret.translation().y() = pos.y();
    ret.translation().z() = pos.z();
    tf2::Quaternion orie = q1.slerp(q2, ration);
    Eigen::Quaterniond orie_q(orie.w(), orie.x(), orie.y(), orie.z());
    ret.linear() = orie_q.toRotationMatrix();
    return ret;
  }
  return std::nullopt;
}

json ImageJson(int n, const sensor_msgs::msg::CompressedImage &image,
               const Eigen::Affine3d &Twl) {
  json c1;

  // Intrinsic parameters
  c1["camera_name"] = "c1";
  c1["camera_model"] = "pinhole";
  c1["fx"] = 1300.83;
  c1["fy"] = 1301.05;
  c1["cx"] = 943.129;
  c1["cy"] = 668.002;
  c1["k1"] = -0.336151;
  c1["k2"] = 0.106222;
  c1["p1"] = -0.004046;
  c1["p2"] = 0.000108;

  // Image location & save
  std::stringstream ss;
  ss << "images/" << std::setw(3) << std::setfill('0') << n << ".png";
  c1["image_url"] = ss.str();
  cv::imwrite("json/" + ss.str(), cv_bridge::toCvCopy(image, "bgr8")->image);

  // Time Stamp
  c1["timestamp"] = rclcpp::Time(image.header.stamp).seconds();

  // Camera pose
  Eigen::Affine3d Twc = Twl * Tlc;
  c1["position"]["x"] = Twc.translation()[0];
  c1["position"]["y"] = Twc.translation()[1];
  c1["position"]["z"] = Twc.translation()[2];
  Eigen::Quaterniond q(Twc.rotation());
  c1["heading"]["x"] = q.x();
  c1["heading"]["y"] = q.y();
  c1["heading"]["z"] = q.z();
  c1["heading"]["w"] = q.w();

  return c1;
}

json CloudJson(sensor_msgs::msg::PointCloud2::SharedPtr cloud,
               const Eigen::Affine3d &Twl) {
  json points;
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
  pcl::fromROSMsg(*cloud, pcl_cloud);
  for (auto &point : pcl_cloud.points) {
    json p;
    Eigen::Vector3d v(point.x, point.y, point.z);
    v = Twl * v;
    p["x"] = v(0);
    p["y"] = v(1);
    p["z"] = v(2);
    // p["intensity"] = point.intensity;
    points.push_back(p);
  }
  return points;
}

void Process(const sensor_msgs::msg::CompressedImage &image) {
  static int n = 0;
  if (cloud && rclcpp::Time(cloud->header.stamp) < image.header.stamp) {
    json j;
    auto left = (rclcpp::Time(cloud->header.stamp) - images.back().header.stamp).nanoseconds();
    auto right = (rclcpp::Time(image.header.stamp) - cloud->header.stamp).nanoseconds();

    // Localization: LiDAR pose
    auto Twl = GetPose(cloud->header.stamp);
    if (!Twl)
      return;

    auto &nearest_image = left < right ? images.back() : image;
    j["images"].push_back(ImageJson(n, nearest_image, Twl.value()));
    j["timestamp"] = rclcpp::Time(cloud->header.stamp).seconds();
    j["points"] = CloudJson(cloud, Twl.value());
    j["device_position"]["x"] = Twl.value().translation()[0];
    j["device_position"]["y"] = Twl.value().translation()[1];
    j["device_position"]["z"] = Twl.value().translation()[2];
    Eigen::Quaterniond q(Twl.value().rotation());
    j["device_heading"]["x"] = q.x();
    j["device_heading"]["y"] = q.y();
    j["device_heading"]["z"] = q.z();
    j["device_heading"]["w"] = q.w();

    std::stringstream ss;
    ss << std::setw(3) << std::setfill('0') << n;
    std::ofstream("json/" + ss.str() + ".json")
        << std::setw(4) << j << std::endl;

    images.clear();
    cloud = nullptr;
    ++n;
  }
  images.push_back(image);
}

Eigen::Affine3d MakeTransform(const nav_msgs::msg::Odometry &msg) {
  Eigen::Affine3d tf;
  tf.translation() = Eigen::Vector3d(msg.pose.pose.position.x,
                                    msg.pose.pose.position.y,
                                    msg.pose.pose.position.z);
  tf.linear() = Eigen::Quaterniond(msg.pose.pose.orientation.w,
                                   msg.pose.pose.orientation.x,
                                   msg.pose.pose.orientation.y,
                                   msg.pose.pose.orientation.z)
                   .toRotationMatrix();
  return tf;
}

int main(int argc, char **argv) {
  if (argc < 2) {
    RCLCPP_ERROR(rclcpp::get_logger("readbag"),
                 "Usage: ros2 run readbag deepenai <bag_file>");
    RCLCPP_ERROR(
        rclcpp::get_logger("readbag"),
        "Note that the output folders will be saved in the working directory.");
    return 1;
  }

  CreateDirectory("json");
  CreateDirectory("json/images");

  rosbag2_cpp::Reader reader_odom;
  reader_odom.open(argv[1]);
  rclcpp::Serialization<nav_msgs::msg::Odometry> odom_ser;
  bool is_first = true;
  Eigen::Affine3d init_tf;
  std::cerr << "Reading Odometries...";
  while (reader_odom.has_next()) {
    rosbag2_storage::SerializedBagMessageSharedPtr msg =
        reader_odom.read_next();
    if (msg->topic_name == "/aeva/AEVA/odometry") {
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      nav_msgs::msg::Odometry odom_msg;
      odom_ser.deserialize_message(&serialized_msg, &odom_msg);
      odom_msg.header.frame_id = "map";
      odom_msg.header.stamp = rclcpp::Time(msg.get()->time_stamp);
      odom_msg.child_frame_id = "aeva/AEVA/sensor";
      if (is_first) {
        is_first = false;
        init_tf = MakeTransform(odom_msg);
      }
      auto current_tf = MakeTransform(odom_msg);
      current_tf = init_tf.inverse() * current_tf;
      Eigen::Quaterniond q(current_tf.linear().w(),
                           current_tf.linear().x(),
                           current_tf.linear().y(),
                           current_tf.linear().z());
      double yaw = current_tf.linear().eulerAngles(0, 1, 2)[2];
      if (std::fabs(current_tf.linear().eulerAngles(0, 1, 2)[0]) > 1.57) {
        yaw = -yaw;
      }
      // current_tf = curr

      // odom_msg.pose.pose.position.x = current_tf.getOrigin().getX();
      // odom_msg.pose.pose.position.y = current_tf.getOrigin().getY();
      // odom_msg.pose.pose.position.z = current_tf.getOrigin().getZ();
      // odom_msg.pose.pose.orientation.x = current_tf.getRotation().getX();
      // odom_msg.pose.pose.orientation.y = current_tf.getRotation().getY();
      // odom_msg.pose.pose.orientation.z = current_tf.getRotation().getZ();
      // odom_msg.pose.pose.orientation.w = current_tf.getRotation().getW();
      odoms.push_back(odom_msg);
    }
  }
  reader_odom.close();
  std::cerr << " Done" << std::endl;

  // rosbag2_cpp::Reader reader;
  // reader.open(argv[1]);
  // rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_ser;
  // rclcpp::Serialization<sensor_msgs::msg::CompressedImage> img_ser;
  // while (reader.has_next()) {
  //   rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();
  //   if (msg->topic_name == "/aeva/AEVA/point_cloud_compensated") {
  //     rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
  //     sensor_msgs::msg::PointCloud2 cloud_msg;
  //     pc_ser.deserialize_message(&serialized_msg, &cloud_msg);
  //     cloud_msg.header.stamp = rclcpp::Time(msg.get()->time_stamp);
  //     cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(cloud_msg);
  //   } else if (msg->topic_name == "/image_raw/compressed") {
  //     rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
  //     sensor_msgs::msg::CompressedImage img_msg;
  //     img_ser.deserialize_message(&serialized_msg, &img_msg);
  //     img_msg.header.stamp = rclcpp::Time(msg.get()->time_stamp);
  //     Process(img_msg);
  //   }
  // }
  // reader.close();
}
