#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sstream>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class IMUSteeringControlNode : public rclcpp::Node
{
  public:
    IMUSteeringControlNode() : Node("imu_steering_control_node")
    {
        // 파라미터 선언 및 기본값 설정
        this->declare_parameter("kp", 2.0);
        this->declare_parameter("ki", 0.1);
        this->declare_parameter("kd", 0.3);
        this->declare_parameter("max_steer", 1.0);
        this->declare_parameter("control_frequency", 50.0);
        this->declare_parameter("angle_tolerance", 0.05); // 약 3도

        // PID 게인 파라미터 로드
        kp_ = this->get_parameter("kp").as_double();
        ki_ = this->get_parameter("ki").as_double();
        kd_ = this->get_parameter("kd").as_double();
        max_steer_ = this->get_parameter("max_steer").as_double();
        control_frequency_ = this->get_parameter("control_frequency").as_double();
        angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();

        // 상태 변수 초기화
        control_enabled_ = false;
        target_angle_ = 0.0;
        current_yaw_ = 0.0;
        imu_received_ = false;
        last_imu_time_ = this->now();

        // PID 제어 변수 초기화
        error_prev_ = 0.0;
        error_integral_ = 0.0;
        last_control_time_ = this->now();
        current_steer_ = 0.0;
        current_throttle_ = 0.0;

        // Publishers
        control_pub_ = this->create_publisher<std_msgs::msg::String>("/dss/SetControl", 1);

        // Subscribers
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/dss/imu", 10, std::bind(&IMUSteeringControlNode::imu_callback, this, std::placeholders::_1));

        target_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/steering/target_angle", 10,
            std::bind(&IMUSteeringControlNode::target_angle_callback, this, std::placeholders::_1));

        enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/steering/enable", 10, std::bind(&IMUSteeringControlNode::enable_callback, this, std::placeholders::_1));

        throttle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/car/throttle", 10, std::bind(&IMUSteeringControlNode::throttle_callback, this, std::placeholders::_1));

        // 제어 타이머 (50Hz)
        double timer_period = 1.0 / control_frequency_;
        control_timer_ = this->create_wall_timer(std::chrono::duration<double>(timer_period),
                                                 std::bind(&IMUSteeringControlNode::control_timer_callback, this));

        // 상태 출력 타이머 (1Hz)
        status_timer_ = this->create_wall_timer(1s, std::bind(&IMUSteeringControlNode::status_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "🧭 IMU Steering Control Node Started");
        RCLCPP_INFO(this->get_logger(), "PID Gains - Kp: %.3f, Ki: %.3f, Kd: %.3f", kp_, ki_, kd_);
        RCLCPP_INFO(this->get_logger(), "Max Steer: %.2f, Tolerance: %.3f rad", max_steer_, angle_tolerance_);
        RCLCPP_INFO(this->get_logger(), "⏳ Waiting for IMU data...");
    }

  private:
    // ROS2 Publishers/Subscribers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throttle_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // 제어 상태
    bool control_enabled_;
    double target_angle_; // 목표 방향각 (rad)
    double current_yaw_;  // 현재 방향각 (rad)
    bool imu_received_;
    rclcpp::Time last_imu_time_;
    double current_throttle_; // 현재 스로틀 값

    // PID 제어 변수
    double kp_, ki_, kd_;
    double max_steer_;
    double control_frequency_;
    double angle_tolerance_;
    double error_prev_;
    double error_integral_;
    rclcpp::Time last_control_time_;
    double current_steer_;

    /**
     * IMU 콜백 - Quaternion을 Yaw 각도로 변환
     */
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        try
        {
            // Quaternion을 tf2로 변환
            tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

            // Roll, Pitch, Yaw 추출
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            current_yaw_ = yaw;
            imu_received_ = true;
            last_imu_time_ = this->now();

            // 주기적으로 IMU 데이터 로그 출력 (2Hz)
            static int imu_log_counter = 0;
            if (++imu_log_counter % 25 == 0) // 50Hz -> 2Hz
            {
                RCLCPP_INFO(this->get_logger(), "📊 IMU Data - Yaw: %.3f rad (%.1f°)", current_yaw_,
                            current_yaw_ * 180.0 / M_PI);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ IMU processing error: %s", e.what());
        }
    }

    /**
     * 목표 각도 설정 콜백
     */
    void target_angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        double new_target = static_cast<double>(msg->data);

        // 각도 정규화 (-π ~ π)
        new_target = normalize_angle(new_target);

        // 목표 각도가 실제로 변경된 경우만 로그 출력
        if (std::abs(new_target - target_angle_) > 0.01)
        {
            RCLCPP_INFO(this->get_logger(), "🎯 Target angle changed: %.3f -> %.3f rad (%.1f° -> %.1f°)", target_angle_,
                        new_target, target_angle_ * 180.0 / M_PI, new_target * 180.0 / M_PI);

            // PID 상태 초기화 (목표 각도 변경 시)
            error_prev_ = 0.0;
            error_integral_ = 0.0;
            last_control_time_ = this->now();
        }

        target_angle_ = new_target;
    }

    /**
     * 제어 활성화/비활성화 콜백
     */
    void enable_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        bool prev_state = control_enabled_;
        control_enabled_ = msg->data;

        if (control_enabled_ && !prev_state)
        {
            RCLCPP_INFO(this->get_logger(), "✅ IMU Steering Control ENABLED - Target: %.3f rad (%.1f°)", target_angle_,
                        target_angle_ * 180.0 / M_PI);
            // PID 상태 초기화
            error_prev_ = 0.0;
            error_integral_ = 0.0;
            last_control_time_ = this->now();
            current_steer_ = 0.0;
        }
        else if (!control_enabled_ && prev_state)
        {
            RCLCPP_INFO(this->get_logger(), "❌ IMU Steering Control DISABLED");
            // 안전을 위해 조향 및 스로틀 초기화
            current_steer_ = 0.0;
            current_throttle_ = 0.0;
            send_control_command(0.0, 0.0);
        }
    }

    /**
     * 상태 출력 타이머 콜백 (1Hz)
     */
    void status_timer_callback()
    {
        // IMU 데이터 수신 상태 확인
        auto time_since_last_imu = (this->now() - last_imu_time_).seconds();

        if (!imu_received_)
        {
            RCLCPP_WARN(this->get_logger(), "⚠️  No IMU data received yet");
        }
        else if (time_since_last_imu > 2.0)
        {
            RCLCPP_WARN(this->get_logger(), "⚠️  No IMU data for %.1f seconds", time_since_last_imu);
        }

        // 제어 상태 출력
        if (control_enabled_)
        {
            double error = angle_difference(target_angle_, current_yaw_);
            RCLCPP_INFO(this->get_logger(),
                        "🧭 STEERING ACTIVE - Target: %.3f rad (%.1f°) | Current: %.3f rad (%.1f°) | Error: %.3f rad "
                        "(%.1f°) | Steer: %.3f",
                        target_angle_, target_angle_ * 180.0 / M_PI, current_yaw_, current_yaw_ * 180.0 / M_PI, error,
                        error * 180.0 / M_PI, current_steer_);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),
                        "🧭 STEERING INACTIVE - Current Yaw: %.3f rad (%.1f°) | Target: %.3f rad (%.1f°)", current_yaw_,
                        current_yaw_ * 180.0 / M_PI, target_angle_, target_angle_ * 180.0 / M_PI);
        }
    }

    /**
     * 제어 타이머 콜백 (50Hz)
     */
    void control_timer_callback()
    {
        if (!control_enabled_)
        {
            current_steer_ = 0.0;
            return;
        }

        // IMU 데이터 수신 확인
        if (!imu_received_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "⚠️  Cannot control - no IMU data received");
            return;
        }

        // PID 제어 계산
        compute_pid_control();

        // 차량 제어 명령 전송 (조향 + 스로틀)
        send_control_command(current_steer_, current_throttle_);
    }

    /**
     * PID 제어 계산
     */
    void compute_pid_control()
    {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_control_time_).seconds();

        // dt가 너무 작거나 큰 경우 보호
        if (dt <= 0.001 || dt > 1.0)
        {
            dt = 1.0 / control_frequency_;
        }

        // 각도 오차 계산 (최단 경로)
        double error = angle_difference(target_angle_, current_yaw_);

        // 허용 오차 내에 있으면 제어 중단
        if (std::abs(error) < angle_tolerance_)
        {
            current_steer_ = 0.0;
            error_integral_ = 0.0; // 적분 리셋
            error_prev_ = error;
            last_control_time_ = current_time;
            return;
        }

        // 적분 계산 (windup 방지)
        error_integral_ += error * dt;
        if (error_integral_ > 1.0)
            error_integral_ = 1.0;
        if (error_integral_ < -1.0)
            error_integral_ = -1.0;

        // 미분 계산
        double error_derivative = (error - error_prev_) / dt;

        // PID 출력 계산
        double pid_output = kp_ * error + ki_ * error_integral_ + kd_ * error_derivative;

        // 조향값 제한 (-1.0 ~ 1.0)
        current_steer_ = std::max(-max_steer_, std::min(max_steer_, pid_output));

        // 다음 계산을 위한 값 저장
        error_prev_ = error;
        last_control_time_ = current_time;

        // 자세한 제어 로그 (5Hz)
        static int control_log_counter = 0;
        if (++control_log_counter % (static_cast<int>(control_frequency_) / 5) == 0)
        {
            RCLCPP_INFO(this->get_logger(),
                        "🎛️  PID Control - Error: %.4f rad (%.2f°), P: %.4f, I: %.4f, D: %.4f, Output: %.4f", error,
                        error * 180.0 / M_PI, kp_ * error, ki_ * error_integral_, kd_ * error_derivative, pid_output);
        }
    }

    /**
     * 스로틀 값 수신 콜백
     */
    void throttle_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        current_throttle_ = static_cast<double>(msg->data);

        // 스로틀 값 로그 (2초마다)
        static int throttle_log_counter = 0;
        if (++throttle_log_counter % (static_cast<int>(control_frequency_) * 2) == 0)
        {
            RCLCPP_INFO(this->get_logger(), "⚡ Throttle updated: %.3f", current_throttle_);
        }
    }

    /**
     * 차량 제어 명령 전송 (조향 + 스로틀)
     */
    void send_control_command(double steer, double throttle)
    {
        try
        {
            // 간단한 문자열 조작으로 JSON 생성
            std::ostringstream json_stream;
            json_stream << std::fixed << std::setprecision(4);
            json_stream << "{"
                        << "\"steer\":" << steer << ","
                        << "\"throttle\":" << throttle << ","
                        << "\"brake\":0.0,"
                        << "\"parkBrake\":false,"
                        << "\"targetGear\":1,"
                        << "\"headLight\":false,"
                        << "\"tailLight\":false,"
                        << "\"turnSignal\":0,"
                        << "\"horn\":false,"
                        << "\"lightMode\":0,"
                        << "\"wiperMode\":0"
                        << "}";

            auto control_msg = std_msgs::msg::String();
            control_msg.data = json_stream.str();
            control_pub_->publish(control_msg);

            // 제어 명령 로그 (5Hz)
            static int cmd_log_counter = 0;
            if (control_enabled_ && ++cmd_log_counter % (static_cast<int>(control_frequency_) / 5) == 0)
            {
                RCLCPP_INFO(this->get_logger(), "🚀 Control Command: steer=%.4f, throttle=%.3f", steer, throttle);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to send control command: %s", e.what());
        }
    }

    /**
     * 각도 정규화 (-π ~ π)
     */
    double normalize_angle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    /**
     * 두 각도 간의 최단 거리 차이 계산
     */
    double angle_difference(double target, double current)
    {
        double diff = target - current;
        return normalize_angle(diff);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<IMUSteeringControlNode>();
        RCLCPP_INFO(node->get_logger(), "🚀 Starting IMU steering control node...");
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        std::cerr << "❌ Exception in IMU steering control node: " << e.what() << std::endl;
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
