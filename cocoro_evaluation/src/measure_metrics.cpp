
#include <chrono>
#include <cstdio>
#include <memory>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sstream>
#include <std_msgs/msg/detail/float64__struct.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int64.hpp>
#include <statistics_msgs/msg/metrics_message.hpp>
#include <cocoro_evaluation/msg/float32_stamped.hpp>
#include <cocoro_evaluation/msg/int64_stamped.hpp>


#include <fstream>
#include <string>

using namespace nav_msgs::msg;
using namespace std_msgs::msg;
using namespace cocoro_evaluation::msg;

class MainNode : public rclcpp::Node
{
public:
  MainNode() : Node("measure_metrics") {
    this->declare_parameter("odom_topic", "/basalt/odom");


    this->get_parameter("odom_topic", odom_topic_);

    odom_sub_ = this->create_subscription<Odometry>(odom_topic_.c_str(), rclcpp::SensorDataQoS(), std::bind(&MainNode::odom_callback, this, std::placeholders::_1));

    cpu_load_pub_ = this->create_publisher<Float32Stamped>("/metrics/cpu_load", rclcpp::SensorDataQoS());

    ram_usage_pub_ = this->create_publisher<Int64Stamped>("/metrics/memory_usage", rclcpp::SensorDataQoS());

    odom_processing_time_pub_ = this->create_publisher<Int64Stamped>("/metrics/odom_processing_time", rclcpp::SensorDataQoS());

    poll_clock_ = this->create_wall_timer(std::chrono::milliseconds(500),
      std::bind(&MainNode::poll_callback, this)
    );

    std::ostringstream oss;
    RCLCPP_INFO_STREAM(this->get_logger(), 
    "Metric Measure Node Started with topics:\n" << 
    "\tAverage CPU load (percentage): " << cpu_load_pub_->get_topic_name() << "\n" << 
    "\tMemory usage (mB): " << ram_usage_pub_->get_topic_name() << "\n" << 
    "\tOdom processing time(nanoseconds 1e-9): " << odom_processing_time_pub_->get_topic_name() << "\n"
   );

  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<Float32Stamped>::SharedPtr cpu_load_pub_;
  rclcpp::Publisher<Int64Stamped>::SharedPtr ram_usage_pub_;
  rclcpp::Publisher<Int64Stamped>::SharedPtr odom_processing_time_pub_;

  rclcpp::TimerBase::SharedPtr poll_clock_;

  std::string odom_topic_;

  double odometry_offset_array[5];
  char odom_array_count = 0;

  float read_cpu_usage_percent() {
    static long prev_total = 0;
    static long prev_idle = 0;

    std::ifstream file("/proc/stat");
    std::string cpu;
    long user, nice, system, idle, iowait, irq, softirq, steal;

    file >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;

    long idle_time = idle + iowait;
    long total_time = user + nice + system + idle + iowait + irq + softirq + steal;

    long total_diff = total_time - prev_total;
    long idle_diff  = idle_time  - prev_idle;

    prev_total = total_time;
    prev_idle  = idle_time;

    if (total_diff == 0) {
      return 0.0;
    }

    return 100.0 * (1.0 - (double)idle_diff / total_diff);
  }

  long read_ram_used_mb()
  {
    std::ifstream file("/proc/meminfo");
    std::string key;
    long value;
    std::string unit;

    long mem_total = 0;
    long mem_available = 0;

    while (file >> key >> value >> unit) {
      if (key == "MemTotal:") {
        mem_total = value;
      } else if (key == "MemAvailable:") {
        mem_available = value;
      }
    }

    return (mem_total - mem_available) / 1024;
  }



  void odom_callback(const Odometry::SharedPtr odom){
    rclcpp::Time now = this->get_clock()->now();
    int64_t now_ns = now.nanoseconds();

    int64_t odom_ns =
        static_cast<int64_t>(odom->header.stamp.sec) * 1000000000LL +
        static_cast<int64_t>(odom->header.stamp.nanosec);

    int64_t diff_ns = now_ns - odom_ns;

    Int64Stamped msg{};
    msg.data = diff_ns;
    msg.header.stamp = now;

    odom_processing_time_pub_->publish(msg);
  }


  void poll_callback() {
    float cpu_usage = read_cpu_usage_percent();
    long mem_usage = read_ram_used_mb();

    auto cpu_usage_ros = Float32Stamped();
    cpu_usage_ros.data = cpu_usage;
    cpu_usage_ros.header.stamp = this->now();

    auto mem_usage_ros = Int64Stamped();
    mem_usage_ros.header.stamp = this->now();
    mem_usage_ros.data = mem_usage;

    cpu_load_pub_->publish(cpu_usage_ros);
    ram_usage_pub_->publish(mem_usage_ros);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto main_node = std::make_shared<MainNode>();
  rclcpp::spin(main_node);
  rclcpp::shutdown();
  return 0;
}
