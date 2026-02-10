#include <chrono>
#include <cstdlib>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <ratio>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>


using namespace sl;

#define imuPub rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr
#define imagePub rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr

class MainNode : public rclcpp::Node {
    
    private:
        Camera zed;
        sl::Mat left_image_zed;
        cv::Mat left_image_ocv;
        sl::Mat right_image_zed;
        cv::Mat right_image_ocv;
        RuntimeParameters runtime_parameters;
        Resolution image_size;
        Resolution new_image_size;

        Timestamp last_imu_ts = 0;

        imuPub imu_pub_;
        imagePub left_pub_;
        imagePub right_pub_;

        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::CallbackGroup::SharedPtr imu_group_;
        rclcpp::CallbackGroup::SharedPtr image_group_;


        std::thread td;
        bool cleanup = false;
        double cam_freq;


        // Mapping between MAT_TYPE and CV_TYPE
        int getOCVtype(sl::MAT_TYPE type) {
            int cv_type = -1;
            switch (type) {
                case MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
                case MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
                case MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
                case MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
                case MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
                case MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
                case MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
                case MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
                default: break;
            }
            return cv_type;
        }

        /**
        * Conversion function between sl::Mat and cv::Mat
        **/
        cv::Mat slMat2cvMat(Mat& input) {
            // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
            // cv::Mat and sl::Mat will share a single memory structure
            return cv::Mat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(MEM::CPU), input.getStepBytes(sl::MEM::CPU));
        }

        void walltimer_cb() {
            // Fetch IMU

            SensorsData sensors_data;
            zed.getSensorsData(sensors_data, TIME_REFERENCE::CURRENT);
            

            if (sensors_data.imu.timestamp > last_imu_ts)
            {
                std_msgs::msg::Header header;
                header.stamp = this->now();
                header.frame_id = "zed_imu";

                sensor_msgs::msg::Imu imu_msg;
                imu_msg.header = header;

                auto imu_data = sensors_data.imu;

                imu_msg.angular_velocity.x = imu_data.angular_velocity_uncalibrated.x;
                imu_msg.angular_velocity.y = imu_data.angular_velocity_uncalibrated.y;
                imu_msg.angular_velocity.z = imu_data.angular_velocity_uncalibrated.z;

                imu_msg.linear_acceleration.x = imu_data.linear_acceleration_uncalibrated.x;
                imu_msg.linear_acceleration.y = imu_data.linear_acceleration_uncalibrated.y;
                imu_msg.linear_acceleration.z = imu_data.linear_acceleration_uncalibrated.z;

                imu_pub_->publish(imu_msg);

                last_imu_ts = sensors_data.imu.timestamp;
            }
        }

        void camera_blocking_loop() {
            using clock = std::chrono::steady_clock;
            auto next = clock::now();

            const auto period = std::chrono::milliseconds((int)cam_freq); 

            while (!cleanup && rclcpp::ok()) {

                //next += period;

                if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS) {
                    
                    zed.retrieveImage(left_image_zed, VIEW::LEFT, MEM::CPU, new_image_size);
                    zed.retrieveImage(right_image_zed, VIEW::LEFT_UNRECTIFIED, MEM::CPU, new_image_size);

                    std_msgs::msg::Header header;
                    header.stamp = this->now();
                    header.frame_id = "zed_camera";

                    auto left_msg = cv_bridge::CvImage(header, "bgra8", left_image_ocv).toImageMsg();
                    auto right_msg = cv_bridge::CvImage(header, "bgra8", right_image_ocv).toImageMsg();

                    left_pub_->publish(*left_msg);
                    right_pub_->publish(*right_msg);
                }

                //std::this_thread::sleep_until(next);
            }
        }



    public:
        MainNode() : rclcpp::Node("ZED_minimal_wrapper") {
            this->declare_parameter("ImuFreq", 200.0);
            double freq = this->get_parameter("ImuFreq").as_double();
            double milli = 1000 / freq;
            if(freq > 400.0 || freq < 10.0) {
                RCLCPP_ERROR_STREAM(get_logger(), "IMU frequency not in range [10.0, 400.0]. Setting default frequency of 200HZ");
                milli = 5.0;
            }

            this->declare_parameter("CamFreq", 30.0);
            cam_freq = this->get_parameter("CamFreq").as_double();
            if(cam_freq < 1.0 || cam_freq > 30.0) {
                RCLCPP_ERROR_STREAM(get_logger(), "IMU frequency not in range [1.0, 30.0]. Setting default frequency of 30HZ");
                cam_freq = 30.0;
            }
            cam_freq = 1000 / cam_freq;
            // ---------------- ZED INIT ----------------
            InitParameters init_params;
            init_params.camera_resolution = RESOLUTION::HD1080;
            init_params.camera_fps = cam_freq;
            init_params.sensors_required = true;
            init_params.camera_disable_self_calib = false;
            init_params.depth_mode = DEPTH_MODE::NONE;
            init_params.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
            init_params.coordinate_units = UNIT::METER;

            ERROR_CODE err = zed.open(init_params);
            if (err != ERROR_CODE::SUCCESS) {
                printf("%s\n", toString(err).c_str());
                zed.close();
                exit(1);
            }

            runtime_parameters.enable_depth = false;

            image_size = zed.getCameraInformation().camera_configuration.resolution;

            int new_width = image_size.width / 2;
            int new_height = image_size.height / 2;
            new_image_size = Resolution(new_width, new_height);

            left_image_zed = sl::Mat(new_width, new_height, MAT_TYPE::U8_C4);
            left_image_ocv = slMat2cvMat(left_image_zed);

            right_image_zed = sl::Mat(new_width, new_height, MAT_TYPE::U8_C4);
            right_image_ocv = slMat2cvMat(right_image_zed);

            // ---------------- ROS INIT ----------------

            rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepAll())
                    .reliable()
                    .durability_volatile();

            left_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/zed/left_img", qos);

            right_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/zed/right_img", qos);

            imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/zed/imu", qos);

            imu_group_ = this->create_callback_group(
                rclcpp::CallbackGroupType::Reentrant);

            image_group_ = this->create_callback_group(
                rclcpp::CallbackGroupType::Reentrant);

            timer_ = this->create_wall_timer(std::chrono::milliseconds((int)milli), std::bind(&MainNode::walltimer_cb, this), imu_group_);
            
            td = std::thread(&MainNode::camera_blocking_loop, this);

            RCLCPP_INFO(this->get_logger(), "Starting minimal zed node!");
        }

        ~MainNode() {
            cleanup = true;
            td.join();
            if(zed.isOpened()) zed.close();
            std::cout << "Exiting wrapper!\n";
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MainNode>();

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    exec.spin();

    return 0;
}



