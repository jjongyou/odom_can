#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

#include <memory>

namespace ichthus
{

class CANOdometry : public rclcpp::Node
{
  private:
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr odom_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mps_pub;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr esp_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sas_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr whl_spd_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mcm_status_sub;

    bool mcm_override;

    float str_ratio;
    float wheel_base;
    float to_rad;

    int esp_msg_len;
    int whl4_spd_len;
    int yaw_rate_idx;

    float ang_vel;

    float curr_ang;
    float curr_mps;
    float curr_theta;
  public:
    explicit CANOdometry(const rclcpp::NodeOptions &);
    ~CANOdometry() override;

    void init_Param();
    void spd_CB(const std_msgs::msg::Float64MultiArray::SharedPtr);
    void yaw_CB(const std_msgs::msg::Float64MultiArray::SharedPtr);
    void ang_CB(const std_msgs::msg::Float64MultiArray::SharedPtr);
    void mcm_status_CB(const std_msgs::msg::Int32::SharedPtr);
};

}
