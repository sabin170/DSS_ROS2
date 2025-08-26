#include <chrono>
#include <memory>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

using json = nlohmann::json;
using namespace std::chrono_literals;

class CruiseControlNode : public rclcpp::Node
{
  public:
    CruiseControlNode() : Node("cruise_control_node")
    {
        // 파라미터 선언 및 기본값 설정
        this->declare_parameter("kp", 0.5);
        this->declare_parameter("ki", 0.1);
        this->declare_parameter("kd", 0.05);
        this->declare_parameter("max_throttle", 0.8);
        this->declare_parameter("max_brake", 0.8);
        this->declare_parameter("control_frequency", 10.0);

        // PID 게인 파라미터 로드
        kp_ = this->get_parameter("kp").as_double();
        ki_ = this->get_parameter("ki").as_double();
        kd_ = this->get_parameter("kd").as_double();
        max_throttle_ = this->get_parameter("max_throttle").as_double();
        max_brake_ = this->get_parameter("max_brake").as_double();
        control_frequency_ = this->get_parameter("control_frequency").as_double();

        // 상태 변수 초기화
        cruise_enabled_ = false;
        target_speed_kph_ = 0.0;
        current_speed_kph_ = 0.0;
        current_speed_ms_ = 0.0;
        engine_rpm_ = 0.0;
        last_snapshot_time_ = this->now();
        snapshot_received_ = false;

        // PID 제어 변수 초기화
        error_prev_ = 0.0;
        error_integral_ = 0.0;
        last_control_time_ = this->now();

        // Publishers
        control_pub_ = this->create_publisher<std_msgs::msg::String>("/dss/SetControl", 1);

        // Subscribers
        snapshot_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/dss/egoVehicle/snapShot", 10,
            std::bind(&CruiseControlNode::snapshot_callback, this, std::placeholders::_1));

        target_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/cruise/target_speed", 10,
            std::bind(&CruiseControlNode::target_speed_callback, this, std::placeholders::_1));

        enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/cruise/enable", 10, std::bind(&CruiseControlNode::enable_callback, this, std::placeholders::_1));

        // 제어 타이머 (10Hz)
        double timer_period = 1.0 / control_frequency_;
        control_timer_ = this->create_wall_timer(std::chrono::duration<double>(timer_period),
                                                 std::bind(&CruiseControlNode::control_timer_callback, this));

        // 상태 출력 타이머 (1Hz)
        status_timer_ = this->create_wall_timer(1s, std::bind(&CruiseControlNode::status_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "🚀 Cruise Control Node Started");
        RCLCPP_INFO(this->get_logger(), "PID Gains - Kp: %.3f, Ki: %.3f, Kd: %.3f", kp_, ki_, kd_);
        RCLCPP_INFO(this->get_logger(), "Max Throttle: %.2f, Max Brake: %.2f", max_throttle_, max_brake_);
        RCLCPP_INFO(this->get_logger(), "⏳ Waiting for vehicle snapShot data...");
    }

  private:
    // ROS2 Publishers/Subscribers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr snapshot_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // 크루즈 컨트롤 상태
    bool cruise_enabled_;
    double target_speed_kph_;
    double current_speed_kph_;
    double current_speed_ms_;
    double engine_rpm_;
    rclcpp::Time last_snapshot_time_;
    bool snapshot_received_;

    // PID 제어 변수
    double kp_, ki_, kd_;
    double max_throttle_, max_brake_;
    double control_frequency_;
    double error_prev_;
    double error_integral_;
    rclcpp::Time last_control_time_;

    // 현재 출력값
    double current_throttle_;
    double current_brake_;

    /**
     * /dss/egoVehicle/snapShot 토픽 콜백 (개선된 버전)
     * JSON 형태의 차량 상태 데이터 파싱
     */
    void snapshot_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        try
        {
            // JSON 데이터 파싱
            json data = json::parse(msg->data);
            
            // 수신 플래그 설정
            snapshot_received_ = true;
            last_snapshot_time_ = this->now();

            // 속도 정보 추출 (더 강건한 방식)
            if (data.contains("speed_kph") && !data["speed_kph"].is_null())
            {
                current_speed_kph_ = data["speed_kph"].get<double>();
            }
            
            if (data.contains("speed_ms") && !data["speed_ms"].is_null())
            {
                current_speed_ms_ = data["speed_ms"].get<double>();
            }
            
            if (data.contains("engine_rpm") && !data["engine_rpm"].is_null())
            {
                engine_rpm_ = data["engine_rpm"].get<double>();
            }

            // 주기적으로 더 자세한 로그 출력 (2Hz)
            static int log_counter = 0;
            if (++log_counter % 50 == 0)  // 100Hz -> 2Hz
            {
                RCLCPP_INFO(this->get_logger(), 
                    "📊 Vehicle Data - Speed: %.2f km/h (%.3f m/s), RPM: %.0f, Data Size: %zu chars",
                    current_speed_kph_, current_speed_ms_, engine_rpm_, msg->data.length());
                    
                // JSON 데이터의 일부를 로그로 출력 (디버깅용)
                std::string data_preview = msg->data.substr(0, 200) + "...";
                RCLCPP_INFO(this->get_logger(), "JSON Preview: %s", data_preview.c_str());
            }
        }
        catch (const json::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to parse snapshot JSON: %s", e.what());
            RCLCPP_ERROR(this->get_logger(), "Raw data preview: %s", msg->data.substr(0, 100).c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Unexpected error in snapshot_callback: %s", e.what());
        }
    }

    /**
     * 상태 출력 타이머 콜백 (1Hz)
     */
    void status_timer_callback()
    {
        // 데이터 수신 상태 확인
        auto time_since_last_snapshot = (this->now() - last_snapshot_time_).seconds();
        
        if (!snapshot_received_)
        {
            RCLCPP_WARN(this->get_logger(), "⚠️  No snapShot data received yet");
        }
        else if (time_since_last_snapshot > 2.0)
        {
            RCLCPP_WARN(this->get_logger(), "⚠️  No snapShot data for %.1f seconds", time_since_last_snapshot);
        }
        
        // 크루즈 컨트롤 상태 출력
        if (cruise_enabled_)
        {
            double error = target_speed_kph_ - current_speed_kph_;
            RCLCPP_INFO(this->get_logger(),
                "🚗 CRUISE ACTIVE - Target: %.1f km/h | Current: %.1f km/h | Error: %.1f | T/B: %.3f/%.3f",
                target_speed_kph_, current_speed_kph_, error, current_throttle_, current_brake_);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),
                "🚗 CRUISE INACTIVE - Current Speed: %.1f km/h | Target: %.1f km/h",
                current_speed_kph_, target_speed_kph_);
        }
    }

    /**
     * 목표 속도 설정 콜백
     */
    void target_speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        double new_target = static_cast<double>(msg->data);

        // 안전 범위 체크 (0-100 km/h)
        if (new_target < 0.0)
        {
            new_target = 0.0;
        }
        else if (new_target > 100.0)
        {
            new_target = 100.0;
        }

        // 목표 속도가 실제로 변경된 경우만 로그 출력
        if (std::abs(new_target - target_speed_kph_) > 0.1)
        {
            RCLCPP_INFO(this->get_logger(), "🎯 Target speed changed: %.2f -> %.2f km/h", 
                target_speed_kph_, new_target);

            // PID 상태 초기화 (목표 속도 변경 시)
            error_prev_ = 0.0;
            error_integral_ = 0.0;
            last_control_time_ = this->now();
        }

        target_speed_kph_ = new_target;
    }

    /**
     * 크루즈 컨트롤 활성화/비활성화 콜백
     */
    void enable_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        bool prev_state = cruise_enabled_;
        cruise_enabled_ = msg->data;

        if (cruise_enabled_ && !prev_state)
        {
            RCLCPP_INFO(this->get_logger(), "✅ Cruise Control ENABLED - Target: %.2f km/h", target_speed_kph_);
            // PID 상태 초기화
            error_prev_ = 0.0;
            error_integral_ = 0.0;
            last_control_time_ = this->now();
            current_throttle_ = 0.0;
            current_brake_ = 0.0;
        }
        else if (!cruise_enabled_ && prev_state)
        {
            RCLCPP_INFO(this->get_logger(), "❌ Cruise Control DISABLED");
            // 안전을 위해 제어 출력 초기화
            current_throttle_ = 0.0;
            current_brake_ = 0.0;
            send_control_command(0.0, 0.0); // 즉시 중립 명령 전송
        }
    }

    /**
     * 제어 타이머 콜백 (10Hz)
     * PID 제어 계산 및 차량 제어 명령 전송
     */
    void control_timer_callback()
    {
        if (!cruise_enabled_)
        {
            current_throttle_ = 0.0;
            current_brake_ = 0.0;
            return;
        }

        // 데이터 수신 확인
        if (!snapshot_received_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "⚠️  Cannot control - no snapShot data received");
            return;
        }

        // PID 제어 계산
        compute_pid_control();

        // 차량 제어 명령 전송
        send_control_command(current_throttle_, current_brake_);
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

        // 속도 오차 계산 (km/h 단위)
        double error = target_speed_kph_ - current_speed_kph_;

        // 적분 계산 (windup 방지)
        error_integral_ += error * dt;
        if (error_integral_ > 50.0)
            error_integral_ = 50.0;
        if (error_integral_ < -50.0)
            error_integral_ = -50.0;

        // 미분 계산
        double error_derivative = (error - error_prev_) / dt;

        // PID 출력 계산
        double pid_output = kp_ * error + ki_ * error_integral_ + kd_ * error_derivative;

        // 스로틀/브레이크 결정 및 제한 (더 민감한 제어)
        if (pid_output > 0.0)
        {
            // 가속 필요 - 저속에서 더 민감하게 반응
            double throttle_scale = (target_speed_kph_ < 10.0) ? 8.0 : 15.0;
            current_throttle_ = std::min(pid_output / throttle_scale, max_throttle_);
            current_brake_ = 0.0;
        }
        else
        {
            // 감속 필요
            current_throttle_ = 0.0;
            double brake_scale = (target_speed_kph_ < 10.0) ? 8.0 : 15.0;
            current_brake_ = std::min(std::abs(pid_output) / brake_scale, max_brake_);
        }

        // 다음 계산을 위한 값 저장
        error_prev_ = error;
        last_control_time_ = current_time;

        // 자세한 제어 로그 (10초마다)
        static int control_log_counter = 0;
        if (++control_log_counter % (static_cast<int>(control_frequency_) * 10) == 0)
        {
            RCLCPP_INFO(this->get_logger(),
                "🎛️  PID Control - Error: %.2f, P: %.3f, I: %.3f, D: %.3f, Output: %.3f",
                error, kp_ * error, ki_ * error_integral_, kd_ * error_derivative, pid_output);
        }
    }

    /**
     * 차량 제어 명령 전송
     */
    void send_control_command(double throttle, double brake)
    {
        try
        {
            json control_json = {
                {"steer", 0.0},
                {"throttle", throttle},
                {"brake", brake},
                {"parkBrake", false},
                {"targetGear", 0},
                {"headLight", false},
                {"tailLight", false},
                {"turnSignal", 0},
                {"horn", false},
                {"lightMode", 0},
                {"wiperMode", 0}
            };

            auto control_msg = std_msgs::msg::String();
            control_msg.data = control_json.dump();
            control_pub_->publish(control_msg);

            // 제어 명령 로그 (필요시)
            static int cmd_log_counter = 0;
            if (cruise_enabled_ && ++cmd_log_counter % (static_cast<int>(control_frequency_) * 5) == 0)
            {
                RCLCPP_INFO(this->get_logger(), "🚀 Control Command - T: %.3f, B: %.3f", throttle, brake);
            }
        }
        catch (const json::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to create control JSON: %s", e.what());
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<CruiseControlNode>();
        RCLCPP_INFO(node->get_logger(), "🚀 Starting cruise control node...");
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        std::cerr << "❌ Exception in cruise control node: " << e.what() << std::endl;
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
