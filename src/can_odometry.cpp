#include <functional>
#include <cmath>

#include "odom_can/can_odometry.hpp"

#define OVERRIDE 2

namespace ichthus
{

CANOdometry::CANOdometry(const rclcpp::NodeOptions & options)
: rclcpp::Node("CAN_Odometry", options)
{
  RCLCPP_INFO(this->get_logger(), "===Start CAN Odometry===");
  init_Param();

  RCLCPP_INFO(this->get_logger(), "Create Pub & Sub");

  odom_pub = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("can_odom", 1);
  mps_pub = this->create_publisher<std_msgs::msg::Float64>("can_mps", 1);

  esp_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "ESP12", 10, std::bind(&CANOdometry::yaw_CB, this, std::placeholders::_1));
  sas_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "SAS11", 10, std::bind(&CANOdometry::ang_CB, this, std::placeholders::_1));
  whl_spd_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "WHL_SPD11", 10, std::bind(&CANOdometry::spd_CB, this, std::placeholders::_1));

  mcm_status_sub = this->create_subscription<std_msgs::msg::Int32>(
    "mcm_status", 10, std::bind(&CANOdometry::mcm_status_CB, this, std::placeholders::_1));

}

CANOdometry::~CANOdometry()
{
}

void CANOdometry::init_Param()
{
  mcm_override = false;

  ang_vel = 0;

  curr_ang = 0;
  curr_mps = 0;
  curr_theta = 0;

  str_ratio = 13.3;
  wheel_base = 2.72;
  to_rad = 0.0174563;

  esp_msg_len = 14;
  whl4_spd_len = 4;
  yaw_rate_idx = 9;
/*
  str_ratio = this->declare_parameter("str_ratio", (float)13.3);
  wheel_base = this->declare_parameter("wheel_base", (float)2.72);
  to_rad = this->declare_parameter("to_rad", (float)0.0174563);

  esp_msg_len = this->declare_parameter("esp_msg_len", (int)14);
  whl4_spd_len = this->declare_parameter("whl4_spd_len", (int)4);
  yaw_rate_idx = this->declare_parameter("yaw_rate_idx", (int)9);

  str_ratio = this->get_parameter("str_ratio").as_double;
  wheel_base = this->get_parameter("wheel_base").as_double;
  to_rad = this->get_parameter("to_rad").as_double;

  esp_msg_len = this->get_parameter("esp_msg_len").as_int;
  whl4_spd_len = this->get_parameter("whl4_spd_len").as_int;
  yaw_rate_idx = this->get_parameter("yaw_rate_idx").as_int;
*/
}


void CANOdometry::yaw_CB(std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!mcm_override)
  {
    int idx = 0;
    float yaw_rate = 0;
    float dt_theta = 0;
    geometry_msgs::msg::TwistWithCovarianceStamped data;

    for (auto esp = msg->data.begin(); idx < esp_msg_len; idx++, esp++)
    {
      if (idx == yaw_rate_idx)
      {
        yaw_rate = *esp;
      }
    }

    dt_theta = yaw_rate * 0.01;
    ang_vel = yaw_rate * to_rad;
    data.twist.twist.linear.x = curr_mps * std::cos(curr_theta);
    data.twist.twist.linear.y = curr_mps * std::sin(curr_theta);
    data.twist.twist.angular.z = ang_vel;
    data.header.stamp = rclcpp::Clock().now();
    odom_pub->publish(data);
    //RCLCPP_INFO(this->get_logger(), "[yaw_CB] dt_theta : %f", dt_theta);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "[yaw_CB] MCM Override");
  }
}

void CANOdometry::spd_CB(std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  int idx = 0;
  float curr_kmph = 0;
  std_msgs::msg::Float64 mps_data;
  for (auto whl_spd = msg->data.begin(); idx < whl4_spd_len; idx++, whl_spd++)
  {
    curr_kmph += *whl_spd;
  }
  curr_kmph /= whl4_spd_len;
  curr_mps = curr_kmph * (1000.0 / 3600.0); 
  mps_data.data = curr_mps;
  mps_pub->publish(mps_data);
  //RCLCPP_INFO(this->get_logger(), "[spd_CB] mps : %f", curr_mps);
}

void CANOdometry::ang_CB(std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!mcm_override)
  {
    int idx = 0;
    int size = 1;
    geometry_msgs::msg::TwistWithCovarianceStamped data;

    for (auto sas_ang = msg->data.begin(); idx < size; idx++, sas_ang++)
    {
      curr_ang += *sas_ang;
    }

    curr_theta = curr_ang / str_ratio;
    curr_ang = 0;
    //RCLCPP_INFO(this->get_logger(), "[ang_CB] curr_theta : %f", curr_theta);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "[ang_CB] MCM Override");
  }
}

void CANOdometry::mcm_status_CB(std_msgs::msg::Int32::SharedPtr msg)
{
  if (msg->data == OVERRIDE)
  {
    mcm_override = false;
  }
  else
  {
    mcm_override = true;
  }
}
} //namespace end
