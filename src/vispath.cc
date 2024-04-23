#include <Eigen/Dense>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class MyNode : public rclcpp::Node {
 public:
  MyNode() : Node("deepenai") {
    rclcpp::QoS qos(rclcpp::KeepLast(10000));
    qos.transient_local();
    pub = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("poses", qos);
  }

  tf2::Transform MakeTransform(const nav_msgs::msg::Odometry &msg) {
    tf2::Transform tf;
    tf.setOrigin(tf2::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y,
                              msg.pose.pose.position.z));
    tf.setRotation(tf2::Quaternion(
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w));
    return tf;
  }

  void Process(std::string filename) {
    rosbag2_cpp::Reader reader;
    reader.open(filename);
    rclcpp::Serialization<nav_msgs::msg::Odometry> odom_ser;
    tf2::Transform init_tf;
    std::cerr << "Reading Odometries...";
    bool is_first = true;
    double x, y, z;
    while (reader.has_next()) {
      rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();
      if (msg->topic_name == "/aeva/AEVA/odometry") {
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        nav_msgs::msg::Odometry odom_msg;
        odom_ser.deserialize_message(&serialized_msg, &odom_msg);
        geometry_msgs::msg::PoseWithCovarianceStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = now();
        if (is_first) {
          x = odom_msg.pose.pose.position.x;
          y = odom_msg.pose.pose.position.y;
          z = odom_msg.pose.pose.position.z;
          is_first = false;
          init_tf = MakeTransform(odom_msg);
        }
        auto current_tf = MakeTransform(odom_msg);
        current_tf = init_tf.inverseTimes(current_tf);
        pose.pose.pose.position.x = current_tf.getOrigin().getX();
        pose.pose.pose.position.y = current_tf.getOrigin().getY();
        pose.pose.pose.position.z = current_tf.getOrigin().getZ();
        pose.pose.pose.orientation.x = current_tf.getRotation().getX();
        pose.pose.pose.orientation.y = current_tf.getRotation().getY();
        pose.pose.pose.orientation.z = current_tf.getRotation().getZ();
        pose.pose.pose.orientation.w = current_tf.getRotation().getW();
        pub->publish(pose);
        rclcpp::Rate(10).sleep();
      }
    }
    reader.close();
    std::cerr << " Done" << std::endl;
  }
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub;
};

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cout << "Usage: ros2 run readbag deepenai <bag_file>" << std::endl;
    return 1;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  node->Process(argv[1]);
  rclcpp::spin(node);
  rclcpp::shutdown();
}
