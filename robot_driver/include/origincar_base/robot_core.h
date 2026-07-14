#ifndef ROBOT_CORE_H_
#define ROBOT_CORE_H_

#include <memory>
#include <string>
#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <origincar_msg/msg/data.hpp>
#include <serial/serial.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define HEADER_BYTE   0x7B
#define TAIL_BYTE     0x7D
#define RX_BUFFER_LEN 24
#define TX_BUFFER_LEN 11

#define GYRO_SCALE    0.00026644f
#define ACCEL_SCALE   1671.84f

extern sensor_msgs::msg::Imu imu_global;

struct Pose2D {
    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;
};

struct Twist2D {
    float vx = 0.0f;
    float vy = 0.0f;
    float omega = 0.0f;
};

struct RawIMU {
    short ax, ay, az;
    short gx, gy, gz;
};

class RobotCore : public rclcpp::Node {
public:
    RobotCore();
    ~RobotCore();

    void run();

private:
    // 回调
    void onTwistCmd(const geometry_msgs::msg::Twist::SharedPtr msg);
    void onAckermannCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

    // 数据处理
    bool fetchSensorFrame();
    void publishOdometry();
    void publishImu();
    void publishVoltage();

    // 辅助工具
    uint8_t calcChecksum(const uint8_t* data, uint8_t len);
    short decodeInt16(uint8_t high, uint8_t low);
    float decodeFloat(uint8_t high, uint8_t low);

    // 成员
    serial::Serial serial_port_;
    Pose2D current_pose_;
    Twist2D current_vel_;
    RawIMU raw_imu_;
    float battery_voltage_ = 0.0f;
    rclcpp::Time last_update_time_;

    // ROS 接口
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ack_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
    rclcpp::Publisher<origincar_msg::msg::Data>::SharedPtr pose_pub_;
    rclcpp::Publisher<origincar_msg::msg::Data>::SharedPtr vel_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 发送缓冲区
    uint8_t tx_buf_[TX_BUFFER_LEN];
    uint8_t rx_buf_[RX_BUFFER_LEN];
    std::string serial_device_;
    int baud_rate_;
    std::string odom_frame_id_, base_frame_id_, imu_frame_id_;
    std::string cmd_vel_topic_, ack_topic_;
    bool use_ackermann_ = false;
};

#endif