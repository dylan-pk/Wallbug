#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>

using Eigen::Vector3d;
using Eigen::Matrix3d;
using std::placeholders::_1;

class CDPRControllerNode : public rclcpp::Node
{
public:
  CDPRControllerNode()
  : Node("cdpr_controller_node")
  {
    // Define anchor positions (fixed in world coordinates)
    anchors_ = {
      {1.0,  0.5, 2.0},
      {-1.0, 0.5, 2.0},
      {-1.0,-0.5, 2.0},
      {1.0, -0.5, 2.0}
    };

    // Define attachment points on the platform (in local coordinates)
    attachments_ = {
      {0.2,  0.2, 0.0},
      {-0.2, 0.2, 0.0},
      {-0.2,-0.2, 0.0},
      {0.2, -0.2, 0.0}
    };

    // Define joint names (one per cable)
    joint_names_ = {"cable_1_joint", "cable_2_joint", "cable_3_joint", "cable_4_joint"};

    // Publishers and subscribers
    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/cdpr/joint_trajectory", 10);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cdpr/target_pose", 10, std::bind(&CDPRControllerNode::poseCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "CDPR Controller Node started.");
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Extract translation
    Vector3d pos(msg->pose.position.x,
                 msg->pose.position.y,
                 msg->pose.position.z);

    // Convert quaternion to rotation matrix
    Eigen::Quaterniond q(
      msg->pose.orientation.w,
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z);
    Matrix3d R = q.normalized().toRotationMatrix();

    // Compute cable lengths
    std::vector<double> lengths;
    for (size_t i = 0; i < anchors_.size(); ++i) {
      Vector3d anchor = anchors_[i];
      Vector3d attach_local = attachments_[i];
      Vector3d attach_world = pos + R * attach_local;
      double length = (anchor - attach_world).norm();
      lengths.push_back(length);
    }

    // Publish as a trajectory message
    trajectory_msgs::msg::JointTrajectory traj_msg;
    traj_msg.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = lengths;
    point.time_from_start = rclcpp::Duration::from_seconds(0.1).to_msg();

    traj_msg.points.push_back(point);

    traj_pub_->publish(traj_msg);

    RCLCPP_INFO(this->get_logger(), "Published cable lengths: [%.3f, %.3f, %.3f, %.3f]",
                lengths[0], lengths[1], lengths[2], lengths[3]);
  }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  std::vector<Vector3d> anchors_;
  std::vector<Vector3d> attachments_;
  std::vector<std::string> joint_names_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CDPRControllerNode>());
  rclcpp::shutdown();
  return 0;
}
