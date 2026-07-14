#ifndef DECISION_NODE_H_
#define DECISION_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <ai_msgs/msg/perception_targets.hpp>
#include <origincar_msg/msg/sign.hpp>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

// 二维码指令
#define QR_NONE     0
#define QR_CW       3   // 顺时针
#define QR_CCW      4   // 逆时针

// 遥操作信号
#define TELEOP_START  5
#define TELEOP_END    6

// 图像参数
const int CAM_W = 640;
const int CAM_H = 480;
const int CENTER_X = CAM_W / 2;

// 任务阶段枚举（顺序打乱）
enum class MissionPhase {
    TRACKING,           // 巡线
    EVADING,            // 避障
    SEARCHING_STRAIGHT, // 直行找线
    RECOVERING_TURN,    // 转向找回
    QR_WAIT             // 二维码停车
};

// 上次避障方向
enum class LastTurn {
    NONE,
    LEFT,
    RIGHT
};

class DecisionNode : public rclcpp::Node {
public:
    DecisionNode();
    ~DecisionNode() = default;

private:
    // 回调
    void onPerception(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);
    void onQrCode(const std_msgs::msg::String::SharedPtr msg);
    void onTeleopSignal(const std_msgs::msg::Int32::SharedPtr msg);

    // 控制辅助
    void sendCmd(double v, double omega);
    void setLineState(bool ok);
    void emergencyStop();

    // 状态处理函数（用于函数指针表）
    void handleTracking();
    void handleEvading();
    void handleSearching();
    void handleRecovering();
    void handleQrWait();

    // 成员变量（更名）
    double cruise_speed_;
    double kp_line_;
    double evade_speed_;
    double cone_detect_thresh_;
    double cone_critical_thresh_;
    double evade_steer_gain_;
    double centered_offset_tolerance_;
    double centered_bias_;  // 1 left, -1 right
    double forward_search_time_;
    double recovery_turn_time_limit_;
    double recovery_speed_ratio_;
    double swing_freq_;

    bool teleop_active_;
    int last_qr_cmd_;
    MissionPhase phase_;
    rclcpp::Time phase_start_time_;
    LastTurn last_turn_dir_;

    // 感知数据缓存（减少重复解析）
    std::vector<const ai_msgs::msg::Roi*> lines_;
    std::vector<const ai_msgs::msg::Roi*> cones_;
    bool threat_detected_;

    // ROS 接口
    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr sub_perception_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_line_state_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_stop_;
    rclcpp::Publisher<origincar_msg::msg::Sign>::SharedPtr pub_sign_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_qr_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_teleop_;

    // 状态函数指针类型
    using StateHandler = void (DecisionNode::*)();
    static const StateHandler state_handlers_[5];
};

#endif