#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>

class ImuRecorder : public rclcpp::Node {
public:
    ImuRecorder() : Node("imu_allan_recorder") {

        // Parameters
        declare_parameter<std::string>("imu_topic", "/zed/zed_node/imu/data_raw");
        declare_parameter<double>("record_hours", 2.0);

        imu_topic_ = get_parameter("imu_topic").as_string();
        record_hours_ = get_parameter("record_hours").as_double();

        record_duration_ =
            std::chrono::duration<double>(record_hours_ * 3600.0);

        // Unique filename
        file_name_ = make_timestamped_filename();

        file_.open(file_name_);

        if (!file_.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open output file!");
            rclcpp::shutdown();
            return;
        }

        file_ << std::fixed << std::setprecision(9);
        file_ << "t,ax,ay,az,gx,gy,gz\n";


        // QoS
        rclcpp::QoS qosi(rclcpp::KeepLast(10));
        qosi.reliable();
        qosi.durability_volatile();

        sub_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_,
            qosi,
            std::bind(&ImuRecorder::callback, this, std::placeholders::_1)
        );

        start_time_ = std::chrono::steady_clock::now();

        // progress timer
        progress_timer_ = create_wall_timer(
            std::chrono::seconds(30),
            std::bind(&ImuRecorder::log_progress, this)
        );

        RCLCPP_INFO(get_logger(),
                    "Recording IMU from %s for %.2f hours",
                    imu_topic_.c_str(),
                    record_hours_);
        RCLCPP_INFO(get_logger(), "Output file: %s", file_name_.c_str());
    }

    ~ImuRecorder() {
        file_.close();
        RCLCPP_INFO(get_logger(),
                    "Recording finished. Total samples: %zu",
                    sample_count_);
    }

private:

    std::string make_timestamped_filename() {
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << "imu_static_"
           << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S")
           << ".csv";
        return ss.str();
    }

    void callback(const sensor_msgs::msg::Imu::SharedPtr msg) {

        auto now = std::chrono::steady_clock::now();

        if (now - start_time_ > record_duration_) {
            RCLCPP_INFO(get_logger(), "Recording duration reached.");
            rclcpp::shutdown();
            return;
        }

        double t =
            msg->header.stamp.sec +
            msg->header.stamp.nanosec * 1e-9;

        file_
            << t << ","
            << msg->linear_acceleration.x << ","
            << msg->linear_acceleration.y << ","
            << msg->linear_acceleration.z << ","
            << msg->angular_velocity.x << ","
            << msg->angular_velocity.y << ","
            << msg->angular_velocity.z << "\n";

        sample_count_++;
    }

    void log_progress() {
        auto elapsed =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start_time_).count();

        double hours = elapsed / 3600.0;
        double percent = (elapsed / record_duration_.count()) * 100.0;

        RCLCPP_INFO(get_logger(),
                    "Progress: %.2f%% | %.2f / %.2f hours | samples: %zu",
                    percent, hours, record_hours_, sample_count_);

        file_.flush();
    }

    // members
    std::string imu_topic_;
    std::string file_name_;

    double record_hours_;
    std::chrono::duration<double> record_duration_;
    std::chrono::steady_clock::time_point start_time_;

    size_t sample_count_ = 0;

    std::ofstream file_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr progress_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuRecorder>());
    rclcpp::shutdown();
    return 0;
}
