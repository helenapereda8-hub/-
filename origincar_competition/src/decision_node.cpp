#include "robot_decision/decision_node.h"
#include <stdexcept>
#include <cmath>

using std::placeholders::_1;

// 初始化静态状态表
const DecisionNode::StateHandler DecisionNode::state_handlers_[5] = {
    &DecisionNode::handleTracking,
    &DecisionNode::handleEvading,
    &DecisionNode::handleSearching,
    &DecisionNode::handleRecovering,
    &DecisionNode::handleQrWait
};

DecisionNode::DecisionNode() : rclcpp::Node("decision_node"),
    teleop_active_(false),
    last_qr_cmd_(QR_NONE),
    phase_(MissionPhase::TRACKING),
    last_turn_dir_(LastTurn::NONE),
    threat_detected_(false)
{
    // 参数声明（默认值与原来一致，但数值微调）
    declare_parameter<double>("cruise_speed", 0.42);
    declare_parameter<double>("kp_line", 0.0030);
    declare_parameter<double>("evade_speed", 0.22);
    declare_parameter<double>("cone_detect_thresh", 225.0);
    declare_parameter<double>("cone_critical_thresh", 310.0);
    declare_parameter<double>("evade_steer_gain", 1.15);
    declare_parameter<double>("centered_tolerance", 35.0);
    declare_parameter<double>("centered_bias", 1.0);
    declare_parameter<double>("forward_search_time", 0.12);
    declare_parameter<double>("recovery_time_limit", 0.0);
    declare_parameter<double>("recovery_speed_ratio", 0.38);
    declare_parameter<double>("swing_freq", 0.3);

    get_parameter("cruise_speed", cruise_speed_);
    get_parameter("kp_line", kp_line_);
    get_parameter("evade_speed", evade_speed_);
    get_parameter("cone_detect_thresh", cone_detect_thresh_);
    get_parameter("cone_critical_thresh", cone_critical_thresh_);
    get_parameter("evade_steer_gain", evade_steer_gain_);
    get_parameter("centered_tolerance", centered_offset_tolerance_);
    get_parameter("centered_bias", centered_bias_);
    get_parameter("forward_search_time", forward_search_time_);
    get_parameter("recovery_time_limit", recovery_turn_time_limit_);
    get_parameter("recovery_speed_ratio", recovery_speed_ratio_);
    get_parameter("swing_freq", swing_freq_);

    RCLCPP_INFO(get_logger(), "DecisionNode started. Centered bias: %.1f", centered_bias_);

    // 订阅与发布
    sub_perception_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
        "origincar_competition", 10, std::bind(&DecisionNode::onPerception, this, _1));
    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pub_line_state_ = create_publisher<std_msgs::msg::Int32>("follower_line", 5);
    pub_stop_ = create_publisher<std_msgs::msg::Int32>("stop", 1);
    pub_sign_ = create_publisher<origincar_msg::msg::Sign>("/sign_switch", 10);
    sub_qr_ = create_subscription<std_msgs::msg::String>(
        "/sign", 10, std::bind(&DecisionNode::onQrCode, this, _1));
    sub_teleop_ = create_subscription<std_msgs::msg::Int32>(
        "/sign4return", 10, std::bind(&DecisionNode::onTeleopSignal, this, _1));
}

// ----- 回调函数 -----
void DecisionNode::onQrCode(const std_msgs::msg::String::SharedPtr msg) {
    origincar_msg::msg::Sign sign_msg;
    std::string raw = msg->data;
    bool ok = false;
    try {
        int num = std::stoi(raw);
        if (num >= 1 && num <= 9999) {
            if (num % 2 != 0) {
                last_qr_cmd_ = QR_CW;
                sign_msg.sign_data = QR_CW;
            } else {
                last_qr_cmd_ = QR_CCW;
                sign_msg.sign_data = QR_CCW;
            }
            pub_sign_->publish(sign_msg);
            ok = true;
        } else {
            RCLCPP_WARN(get_logger(), "QR number %d out of range", num);
            last_qr_cmd_ = QR_NONE;
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Invalid QR data: %s", raw.c_str());
        last_qr_cmd_ = QR_NONE;
    }
    if (ok) {
        RCLCPP_INFO(get_logger(), "QR command %d received. Enter QR_WAIT.", last_qr_cmd_);
        phase_ = MissionPhase::QR_WAIT;
        sendCmd(0.0, 0.0);
    }
}

void DecisionNode::onTeleopSignal(const std_msgs::msg::Int32::SharedPtr msg) {
    if (msg->data == TELEOP_START) {
        RCLCPP_INFO(get_logger(), "Teleop ON");
        teleop_active_ = true;
    } else if (msg->data == TELEOP_END) {
        RCLCPP_INFO(get_logger(), "Teleop OFF. Resume tracking.");
        teleop_active_ = false;
        last_qr_cmd_ = QR_NONE;
        phase_ = MissionPhase::TRACKING;
    }
}

// ----- 控制辅助 -----
void DecisionNode::sendCmd(double v, double omega) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = omega;
    pub_cmd_->publish(cmd);
}

void DecisionNode::setLineState(bool ok) {
    std_msgs::msg::Int32 msg;
    msg.data = ok ? 1 : 0;
    pub_line_state_->publish(msg);
}

void DecisionNode::emergencyStop() {
    std_msgs::msg::Int32 msg;
    msg.data = 0;
    pub_stop_->publish(msg);
    RCLCPP_ERROR(get_logger(), "EMERGENCY STOP!");
}

// ----- 核心感知处理 -----
void DecisionNode::onPerception(const ai_msgs::msg::PerceptionTargets::SharedPtr msg) {
    // 最高优先级
    if (last_qr_cmd_ != QR_NONE) {
        if (phase_ != MissionPhase::QR_WAIT) {
            phase_ = MissionPhase::QR_WAIT;
            sendCmd(0.0, 0.0);
        }
        setLineState(false);
        return;
    }
    if (teleop_active_) {
        setLineState(false);
        return;
    }

    // 清空并填充感知数据
    lines_.clear();
    cones_.clear();
    for (const auto& t : msg->targets) {
        if (t.rois.empty()) continue;
        if (t.type == "line") lines_.push_back(&t.rois[0]);
        else if (t.type == "zt") cones_.push_back(&t.rois[0]);
    }

    // 锥桶威胁检测
    threat_detected_ = false;
    if (phase_ != MissionPhase::QR_WAIT && !teleop_active_) {
        float max_h = 0.0f;
        for (auto* c : cones_) {
            max_h = std::max(max_h, static_cast<float>(c->rect.height));
        }
        if (max_h > cone_detect_thresh_) {
            threat_detected_ = true;
            if (phase_ != MissionPhase::EVADING) {
                RCLCPP_INFO(get_logger(), "Threat detected H=%.1f -> EVADING", max_h);
                phase_ = MissionPhase::EVADING;
                last_turn_dir_ = LastTurn::NONE;
            }
        }
    }

    // 执行状态机（通过函数指针表）
    int idx = static_cast<int>(phase_);
    if (idx >= 0 && idx < 5) {
        (this->*state_handlers_[idx])();
    } else {
        RCLCPP_ERROR(get_logger(), "Invalid phase!");
    }

    // 发布巡线状态
    setLineState(phase_ == MissionPhase::TRACKING);
}

// ----- 各状态处理函数（完全重构逻辑顺序）-----
void DecisionNode::handleTracking() {
    double v = 0.0, omega = 0.0;
    if (threat_detected_) {
        // 威胁已由上层处理，这里不重复做
        return;
    }
    if (!lines_.empty()) {
        // 找最近的线（y_offset最小）
        auto it = std::min_element(lines_.begin(), lines_.end(),
            [](const auto* a, const auto* b) { return a->rect.y_offset < b->rect.y_offset; });
        const auto& roi = *(*it);
        float cx = roi.rect.x_offset + roi.rect.width / 2.0f;
        float err = cx - CENTER_X;
        v = cruise_speed_;
        omega = -kp_line_ * err;
        last_turn_dir_ = LastTurn::NONE;
    } else {
        // 丢线 -> 进入搜索直行
        RCLCPP_WARN(get_logger(), "Lost line -> SEARCHING_STRAIGHT");
        phase_ = MissionPhase::SEARCHING_STRAIGHT;
        phase_start_time_ = now();
        last_turn_dir_ = LastTurn::NONE;
        // 本周期不发送速度，等下次循环进入search处理
        return;
    }
    sendCmd(v, omega);
}

void DecisionNode::handleEvading() {
    if (!threat_detected_ || cones_.empty()) {
        // 威胁消失，尝试结束避障
        RCLCPP_INFO(get_logger(), "Threat gone. Exiting EVADING.");
        if (!lines_.empty()) {
            phase_ = MissionPhase::TRACKING;
            RCLCPP_INFO(get_logger(), "Line found, back to TRACKING.");
        } else {
            // 无线，根据参数决定下一步
            if (forward_search_time_ > 0.0) {
                phase_ = MissionPhase::SEARCHING_STRAIGHT;
                RCLCPP_INFO(get_logger(), "No line -> SEARCHING_STRAIGHT");
            } else {
                phase_ = MissionPhase::RECOVERING_TURN;
                RCLCPP_INFO(get_logger(), "No line and no forward time -> RECOVERING_TURN");
            }
            phase_start_time_ = now();
        }
        return;
    }

    // 找最高的锥桶
    const ai_msgs::msg::Roi* target = nullptr;
    float max_h = 0.0f;
    for (auto* c : cones_) {
        float h = c->rect.height;
        if (h > max_h) { max_h = h; target = c; }
    }
    if (!target) return;

    float cx = target->rect.x_offset + target->rect.width / 2.0f;
    float lateral_err = cx - CENTER_X;
    double v = evade_speed_;
    double omega = 0.0;

    if (std::abs(lateral_err) < centered_offset_tolerance_) {
        // 正前方
        omega = evade_steer_gain_ * centered_bias_;
        last_turn_dir_ = (centered_bias_ > 0) ? LastTurn::LEFT : LastTurn::RIGHT;
        if (max_h > cone_critical_thresh_) {
            v = std::min(v, 0.05);
            omega *= 1.5;
        }
        RCLCPP_DEBUG(get_logger(), "Centered cone, omega=%.2f", omega);
    } else if (lateral_err < 0) {
        // 锥桶在左 -> 右转
        omega = -evade_steer_gain_;
        last_turn_dir_ = LastTurn::RIGHT;
    } else {
        // 锥桶在右 -> 左转
        omega = evade_steer_gain_;
        last_turn_dir_ = LastTurn::LEFT;
    }
    sendCmd(v, omega);
}

void DecisionNode::handleSearching() {
    if (!lines_.empty()) {
        RCLCPP_INFO(get_logger(), "Found line in SEARCHING -> TRACKING");
        phase_ = MissionPhase::TRACKING;
        return;
    }
    double elapsed = (now() - phase_start_time_).seconds();
    if (elapsed < forward_search_time_) {
        double v = cruise_speed_ * recovery_speed_ratio_;
        sendCmd(v, 0.0);
        RCLCPP_DEBUG(get_logger(), "SEARCHING straight %.2f/%.2f", elapsed, forward_search_time_);
    } else {
        RCLCPP_INFO(get_logger(), "SEARCHING timeout -> RECOVERING_TURN");
        phase_ = MissionPhase::RECOVERING_TURN;
        phase_start_time_ = now();
    }
}

void DecisionNode::handleRecovering() {
    if (!lines_.empty()) {
        RCLCPP_INFO(get_logger(), "Found line in RECOVERING -> TRACKING");
        phase_ = MissionPhase::TRACKING;
        return;
    }
    // 检查是否超时（如果设置了时间限制）
    if (recovery_turn_time_limit_ > 0.0) {
        if ((now() - phase_start_time_).seconds() > recovery_turn_time_limit_) {
            RCLCPP_WARN(get_logger(), "Recovery timeout, retry SEARCHING");
            phase_ = MissionPhase::SEARCHING_STRAIGHT;
            phase_start_time_ = now();
            last_turn_dir_ = LastTurn::NONE;
            return;
        }
    }

    double v = 0.8;
    double omega = 0.0;
    if (last_turn_dir_ == LastTurn::LEFT) {
        omega = -1.25;  // 右转恢复
        RCLCPP_DEBUG(get_logger(), "Recover RIGHT (was LEFT)");
    } else if (last_turn_dir_ == LastTurn::RIGHT) {
        omega = 1.25;   // 左转恢复
        RCLCPP_DEBUG(get_logger(), "Recover LEFT (was RIGHT)");
    } else {
        omega = 1.0;    // 无明确方向，左转试探
        RCLCPP_DEBUG(get_logger(), "Recover LEFT (default)");
    }
    sendCmd(v, omega);
}

void DecisionNode::handleQrWait() {
    sendCmd(0.0, 0.0);
    RCLCPP_DEBUG(get_logger(), "QR_WAIT");
}

// ----- 主函数入口 -----
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DecisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}