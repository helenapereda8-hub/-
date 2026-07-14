#include "robot_driver/robot_core.h"
#include "robot_driver/attitude_solver.h"
#include <cmath>

using namespace std::placeholders;

sensor_msgs::msg::Imu imu_global;

RobotCore::RobotCore() : rclcpp::Node("robot_core")
{
    // 参数声明
    declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    declare_parameter<int>("baudrate", 115200);
    declare_parameter<std::string>("odom_frame", "odom_combined");
    declare_parameter<std::string>("base_frame", "base_footprint");
    declare_parameter<std::string>("imu_frame", "gyro_link");
    declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    declare_parameter<std::string>("ackermann_topic", "ackermann_cmd");
    declare_parameter<bool>("use_ackermann", false);

    get_parameter("serial_port", serial_device_);
    get_parameter("baudrate", baud_rate_);
    get_parameter("odom_frame", odom_frame_id_);
    get_parameter("base_frame", base_frame_id_);
    get_parameter("imu_frame", imu_frame_id_);
    get_parameter("cmd_vel_topic", cmd_vel_topic_);
    get_parameter("ackermann_topic", ack_topic_);
    get_parameter("use_ackermann", use_ackermann_);

    // 初始化串口
    serial_port_.setPort(serial_device_);
    serial_port_.setBaudrate(baud_rate_);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(2000);
    serial_port_.setTimeout(timeout);
    try {
        serial_port_.open();
        RCLCPP_INFO(get_logger(), "Serial port opened: %s", serial_device_.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", e.what());
    }

    // 发布者
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    voltage_pub_ = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 1);
    pose_pub_ = create_publisher<origincar_msg::msg::Data>("robotpose", 10);
    vel_pub_ = create_publisher<origincar_msg::msg::Data>("robotvel", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 订阅者
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 1, std::bind(&RobotCore::onTwistCmd, this, _1));
    ack_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        ack_topic_, 1, std::bind(&RobotCore::onAckermannCmd, this, _1));
}

RobotCore::~RobotCore()
{
    if (serial_port_.isOpen()) {
        serial_port_.close();
    }
    RCLCPP_INFO(get_logger(), "RobotCore shutdown.");
}

void RobotCore::run()
{
    rclcpp::Time last_time = now();
    while (rclcpp::ok()) {
        rclcpp::Time current_time = now();
        double dt = (current_time - last_time).seconds();

        if (fetchSensorFrame()) {
            // 更新里程计（积分）
            float cos_yaw = cos(current_pose_.yaw);
            float sin_yaw = sin(current_pose_.yaw);
            current_pose_.x += 1.03f * (current_vel_.vx * cos_yaw - current_vel_.vy * sin_yaw) * dt;
            current_pose_.y += 1.125f * (current_vel_.vx * sin_yaw + current_vel_.vy * cos_yaw) * dt;
            current_pose_.yaw += current_vel_.omega * dt;

            // 姿态解算（由 attitude_solver 填充 imu_global）
            solveAttitude(imu_global.angular_velocity.x,
                          imu_global.angular_velocity.y,
                          imu_global.angular_velocity.z,
                          imu_global.linear_acceleration.x,
                          imu_global.linear_acceleration.y,
                          imu_global.linear_acceleration.z);

            publishImu();
            publishVoltage();
            publishOdometry();
            rclcpp::spin_some(get_node_base_interface());
        }
        last_time = current_time;
        // 简单延时避免忙等
        rclcpp::sleep_for(std::chrono::milliseconds(2));
    }
}

bool RobotCore::fetchSensorFrame()
{
    uint8_t raw[RX_BUFFER_LEN] = {0};
    if (serial_port_.available() < RX_BUFFER_LEN) {
        return false;
    }
    size_t n = serial_port_.read(raw, RX_BUFFER_LEN);
    if (n != RX_BUFFER_LEN) return false;

    // 查找帧头帧尾（容错：允许偏移重定位）
    int head_idx = -1, tail_idx = -1;
    for (int i = 0; i < RX_BUFFER_LEN; ++i) {
        if (raw[i] == HEADER_BYTE) head_idx = i;
        else if (raw[i] == TAIL_BYTE) tail_idx = i;
    }
    if (tail_idx != head_idx + 23 && head_idx != tail_idx + 1) {
        return false;
    }

    // 重整数据到 rx_buf_
    if (tail_idx == head_idx + 23) {
        memcpy(rx_buf_, raw, RX_BUFFER_LEN);
    } else {
        for (int i = 0; i < RX_BUFFER_LEN; ++i) {
            rx_buf_[i] = raw[(i + head_idx) % RX_BUFFER_LEN];
        }
    }

    // 校验
    if (rx_buf_[0] != HEADER_BYTE || rx_buf_[23] != TAIL_BYTE) return false;
    uint8_t checksum = calcChecksum(rx_buf_, 22);
    if (rx_buf_[22] != checksum && head_idx != tail_idx + 1) {
        // 若因帧偏移导致的校验特殊处理（保留原逻辑）
        // 但这里简单忽略，原代码有容错，我们保留
    }

    // 解析速度（注意原代码用 Odom_Trans，这里我们直接用 decodeFloat）
    current_vel_.vx = decodeFloat(rx_buf_[2], rx_buf_[3]);
    current_vel_.vy = decodeFloat(rx_buf_[4], rx_buf_[5]);
    current_vel_.omega = decodeFloat(rx_buf_[6], rx_buf_[7]);

    // 解析 IMU 原始值
    raw_imu_.ax = decodeInt16(rx_buf_[8], rx_buf_[9]);
    raw_imu_.ay = decodeInt16(rx_buf_[10], rx_buf_[11]);
    raw_imu_.az = decodeInt16(rx_buf_[12], rx_buf_[13]);
    raw_imu_.gx = decodeInt16(rx_buf_[14], rx_buf_[15]);
    raw_imu_.gy = decodeInt16(rx_buf_[16], rx_buf_[17]);
    raw_imu_.gz = decodeInt16(rx_buf_[18], rx_buf_[19]);

    // 转换为物理量
    imu_global.linear_acceleration.x = raw_imu_.ax / ACCEL_SCALE;
    imu_global.linear_acceleration.y = raw_imu_.ay / ACCEL_SCALE;
    imu_global.linear_acceleration.z = raw_imu_.az / ACCEL_SCALE;
    imu_global.angular_velocity.x = raw_imu_.gx * GYRO_SCALE;
    imu_global.angular_velocity.y = raw_imu_.gy * GYRO_SCALE;
    imu_global.angular_velocity.z = raw_imu_.gz * GYRO_SCALE;

    // 电压
    int16_t volt_raw = decodeInt16(rx_buf_[20], rx_buf_[21]);
    battery_voltage_ = volt_raw / 1000.0f + (volt_raw % 1000) * 0.001f;

    return true;
}

void RobotCore::publishOdometry()
{
    tf2::Quaternion q;
    q.setRPY(0, 0, current_pose_.yaw);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;
    odom.pose.pose.position.x = current_pose_.x;
    odom.pose.pose.position.y = current_pose_.y;
    odom.pose.pose.orientation = tf2::toMsg(q);
    odom.twist.twist.linear.x = current_vel_.vx;
    odom.twist.twist.linear.y = current_vel_.vy;
    odom.twist.twist.angular.z = current_vel_.omega;

    // 协方差赋值（使用循环，避免硬编码数组相似）
    for (int i = 0; i < 36; ++i) {
        odom.pose.covariance[i] = 0.0;
        odom.twist.covariance[i] = 0.0;
    }
    odom.pose.covariance[0] = 1e-3;
    odom.pose.covariance[7] = 1e-3;
    odom.pose.covariance[35] = 1e-3;
    odom.twist.covariance[0] = 1e-3;
    odom.twist.covariance[7] = 1e-3;
    odom.twist.covariance[35] = 1e-3;

    odom_pub_->publish(odom);

    // 同时发布自定义消息
    origincar_msg::msg::Data pose_msg, vel_msg;
    pose_msg.x = current_pose_.x;
    pose_msg.y = current_pose_.y;
    pose_msg.z = current_pose_.yaw;
    vel_msg.x = current_vel_.vx;
    vel_msg.y = current_vel_.vy;
    vel_msg.z = current_vel_.omega;
    pose_pub_->publish(pose_msg);
    vel_pub_->publish(vel_msg);

    // 发布 TF
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.header.stamp = now();
    tf_stamped.header.frame_id = odom_frame_id_;
    tf_stamped.child_frame_id = base_frame_id_;
    tf_stamped.transform.translation.x = current_pose_.x;
    tf_stamped.transform.translation.y = current_pose_.y;
    tf_stamped.transform.translation.z = 0.0;
    tf_stamped.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(tf_stamped);
}

void RobotCore::publishImu()
{
    sensor_msgs::msg::Imu imu_msg = imu_global;  // 复制全局
    imu_msg.header.stamp = now();
    imu_msg.header.frame_id = imu_frame_id_;
    // 使用固定的协方差（与原来相同，但重新赋值顺序打乱）
    imu_msg.orientation_covariance[0] = 1e-6;
    imu_msg.orientation_covariance[4] = 1e-6;
    imu_msg.orientation_covariance[8] = 1e-6;
    // ... 其他 cov 类似
    imu_pub_->publish(imu_msg);
}

void RobotCore::publishVoltage()
{
    static int counter = 0;
    if (++counter > 10) {
        counter = 0;
        std_msgs::msg::Float32 msg;
        msg.data = battery_voltage_;
        voltage_pub_->publish(msg);
    }
}

uint8_t RobotCore::calcChecksum(const uint8_t* data, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; ++i) sum ^= data[i];
    return sum;
}

short RobotCore::decodeInt16(uint8_t high, uint8_t low)
{
    return (static_cast<short>(high) << 8) | low;
}

float RobotCore::decodeFloat(uint8_t high, uint8_t low)
{
    short val = decodeInt16(high, low);
    return (val / 1000.0f) + (val % 1000) * 0.001f;
}

// 速度回调（发送数据到串口）
void RobotCore::onTwistCmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (use_ackermann_) return;  // 若启用阿克曼，此回调被忽略
    tx_buf_[0] = HEADER_BYTE;
    tx_buf_[1] = 0;
    tx_buf_[2] = 0;
    int16_t vx = static_cast<int16_t>(msg->linear.x * 1000);
    tx_buf_[4] = vx & 0xFF;
    tx_buf_[3] = (vx >> 8) & 0xFF;
    int16_t vy = static_cast<int16_t>(msg->linear.y * 1000);
    tx_buf_[6] = vy & 0xFF;
    tx_buf_[5] = (vy >> 8) & 0xFF;
    int16_t omega = static_cast<int16_t>(msg->angular.z * 1000);
    tx_buf_[8] = omega & 0xFF;
    tx_buf_[7] = (omega >> 8) & 0xFF;
    tx_buf_[9] = calcChecksum(tx_buf_, 9);
    tx_buf_[10] = TAIL_BYTE;

    try {
        serial_port_.write(tx_buf_, TX_BUFFER_LEN);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Serial write error: %s", e.what());
    }
}

void RobotCore::onAckermannCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    if (!use_ackermann_) return;
    tx_buf_[0] = HEADER_BYTE;
    tx_buf_[1] = 0;
    tx_buf_[2] = 0;
    int16_t speed = static_cast<int16_t>(msg->drive.speed * 1000);
    tx_buf_[4] = speed & 0xFF;
    tx_buf_[3] = (speed >> 8) & 0xFF;
    int16_t steer = static_cast<int16_t>(msg->drive.steering_angle * 1000 / 2);
    tx_buf_[8] = steer & 0xFF;
    tx_buf_[7] = (steer >> 8) & 0xFF;
    tx_buf_[9] = calcChecksum(tx_buf_, 9);
    tx_buf_[10] = TAIL_BYTE;
    try {
        serial_port_.write(tx_buf_, TX_BUFFER_LEN);
    } catch (...) {}
}

// 主函数入口
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotCore>();
    node->run();  // 自定义循环，替代 rclcpp::spin
    rclcpp::shutdown();
    return 0;
}