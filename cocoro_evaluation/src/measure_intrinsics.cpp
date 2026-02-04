#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <Eigen/Dense>
#include <vector>
#include <chrono>
#include <iostream>

using namespace std::chrono;

class MeasureIntrinsics : public rclcpp::Node
{
public:
    MeasureIntrinsics()
        : Node("measure_intrinsics"), duration_sec_(300.0)
    {
        sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/zed/imu",
            rclcpp::SensorDataQoS(),
            std::bind(&MeasureIntrinsics::callback, this, std::placeholders::_1));

        start_time_ = steady_clock::now();

        RCLCPP_INFO(get_logger(), "Recording IMU for 5 minutes. Keep sensor perfectly still.");
    }

private:
    void callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        Eigen::Vector3d accel(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);

        Eigen::Vector3d gyro(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);

        accel_data_.push_back(accel);
        gyro_data_.push_back(gyro);

        auto now = steady_clock::now();
        double elapsed = duration<double>(now - start_time_).count();

        if (elapsed >= duration_sec_)
        {
            compute();
            rclcpp::shutdown();
        }
    }

    void compute()
    {
        RCLCPP_INFO(get_logger(), "Processing calibration...");

        size_t N = accel_data_.size();
        if (N < 100)
        {
            RCLCPP_ERROR(get_logger(), "Not enough IMU samples!");
            return;
        }

        // ----- Mean raw accel -----
        Eigen::Vector3d accel_mean_raw = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero();

        for (size_t i = 0; i < N; i++)
        {
            accel_mean_raw += accel_data_[i];
            gyro_mean += gyro_data_[i];
        }

        accel_mean_raw /= N;
        gyro_mean /= N;

        // ----- Estimate gravity direction -----
        Eigen::Vector3d gravity = accel_mean_raw.normalized() * 9.81;

        // ----- True accel bias -----
        Eigen::Vector3d accel_bias = accel_mean_raw - gravity;

        // ----- Variance -----
        Eigen::Vector3d accel_var = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyro_var = Eigen::Vector3d::Zero();

        for (size_t i = 0; i < N; i++)
        {
            Eigen::Vector3d da = (accel_data_[i] - gravity) - accel_bias;
            Eigen::Vector3d dg = gyro_data_[i] - gyro_mean;

            accel_var += da.cwiseProduct(da);
            gyro_var += dg.cwiseProduct(dg);
        }

        accel_var /= N;
        gyro_var /= N;

        Eigen::Vector3d accel_std = accel_var.cwiseSqrt();
        Eigen::Vector3d gyro_std = gyro_var.cwiseSqrt();

        // Bias random walk estimate (simple approx)
        Eigen::Vector3d accel_bias_std = accel_std * 0.1;
        Eigen::Vector3d gyro_bias_std = gyro_std * 0.1;

        double imu_rate = N / duration_sec_;

        // ----- Output -----
        std::cout << "\n===== IMU CALIBRATION RESULT =====\n\n";

        std::cout << "\"calib_accel_bias\": [\n";
        std::cout << "  " << accel_bias.x() << ",\n";
        std::cout << "  " << accel_bias.y() << ",\n";
        std::cout << "  " << accel_bias.z() << ",\n";
        for (int i = 0; i < 6; i++) std::cout << "  0.0,\n";
        std::cout << "],\n\n";

        std::cout << "\"calib_gyro_bias\": [\n";
        std::cout << "  " << gyro_mean.x() << ",\n";
        std::cout << "  " << gyro_mean.y() << ",\n";
        std::cout << "  " << gyro_mean.z() << ",\n";
        for (int i = 0; i < 9; i++) std::cout << "  0.0,\n";
        std::cout << "],\n\n";

        std::cout << "\"imu_update_rate\": " << imu_rate << ",\n";

        std::cout << "\"accel_noise_std\": ["
                  << accel_std.x() << ", "
                  << accel_std.y() << ", "
                  << accel_std.z() << "],\n";

        std::cout << "\"gyro_noise_std\": ["
                  << gyro_std.x() << ", "
                  << gyro_std.y() << ", "
                  << gyro_std.z() << "],\n";

        std::cout << "\"accel_bias_std\": ["
                  << accel_bias_std.x() << ", "
                  << accel_bias_std.y() << ", "
                  << accel_bias_std.z() << "],\n";

        std::cout << "\"gyro_bias_std\": ["
                  << gyro_bias_std.x() << ", "
                  << gyro_bias_std.y() << ", "
                  << gyro_bias_std.z() << "]\n";

        std::cout << "\n==================================\n";
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;

    std::vector<Eigen::Vector3d> accel_data_;
    std::vector<Eigen::Vector3d> gyro_data_;

    steady_clock::time_point start_time_;
    double duration_sec_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MeasureIntrinsics>());
    rclcpp::shutdown();
    return 0;
}
