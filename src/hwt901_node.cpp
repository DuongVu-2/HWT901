// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/imu.hpp>
// #include <geometry_msgs/msg/vector3_stamped.hpp> // Thêm thư viện này
// #include <tf2/LinearMath/Quaternion.h>
// #include "hwt901/hwt901.hpp"

// class ImuPublisherNode : public rclcpp::Node {
// public:
//     ImuPublisherNode() : Node("hwt901_publisher") {
//         std::string port = this->declare_parameter("port", "/dev/imu");
//         int baudrate = this->declare_parameter("baudrate", 115200);

//         // Publisher cho IMU chuẩn (Quaternion)
//         imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        
//         // Publisher cho Góc Euler (Roll, Pitch, Yaw)
//         euler_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/imu/euler", 10);

//         sensor_ = std::make_unique<hwt901::Hwt901Sensor>(port, baudrate);

//         sensor_->set_data_callback([this](const hwt901::ImuData& data) {
//             this->publish_imu_data(data);
//             this->publish_euler_data(data); // Gọi hàm publish góc Euler
//         });

//         sensor_->start();
//         RCLCPP_INFO(this->get_logger(), "Đã bắt đầu node IMU. Topic: /imu/data và /imu/euler");
//     }

// private:
//     void publish_imu_data(const hwt901::ImuData& data) {
//         auto msg = sensor_msgs::msg::Imu();
//         msg.header.stamp = this->now();
//         msg.header.frame_id = "imu_link";

//         msg.linear_acceleration.x = data.accel_x;
//         msg.linear_acceleration.y = data.accel_y;
//         msg.linear_acceleration.z = data.accel_z;

//         msg.angular_velocity.x = data.gyro_x;
//         msg.angular_velocity.y = data.gyro_y;
//         msg.angular_velocity.z = data.gyro_z;

//         // Chuyển Euler sang Quaternion để đúng chuẩn ROS 
//         tf2::Quaternion q;
//         q.setRPY(data.roll, data.pitch, data.yaw);
//         msg.orientation.x = q.x();
//         msg.orientation.y = q.y();
//         msg.orientation.z = q.z();
//         msg.orientation.w = q.w();

//         imu_pub_->publish(msg);
//     }

//     void publish_euler_data(const hwt901::ImuData& data) {
//         auto euler_msg = geometry_msgs::msg::Vector3Stamped();
//         euler_msg.header.stamp = this->now();
//         euler_msg.header.frame_id = "imu_link";

//         // Đơn vị Radian (Hoặc bạn có thể nhân 180/M_PI để ra Độ)
//         euler_msg.vector.x = data.roll * 180 / M_PI;  // Trục X [cite: 167]
//         euler_msg.vector.y = data.pitch * 180 / M_PI; // Trục Y [cite: 168]
//         euler_msg.vector.z = data.yaw * 180 / M_PI;  // Trục Z [cite: 168]

//         euler_pub_->publish(euler_msg);
//     }

//     rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
//     rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;
//     std::unique_ptr<hwt901::Hwt901Sensor> sensor_;
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ImuPublisherNode>());
//     rclcpp::shutdown();
//     return 0;
// }


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "hwt901/hwt901.hpp"

class ImuPublisherNode : public rclcpp::Node {
public:
    ImuPublisherNode() : Node("hwt901_publisher") {
        // 1. Khai báo và đọc tham số từ file YAML (kèm giá trị mặc định phòng hờ)
        std::string port = this->declare_parameter("port", "/dev/imu");
        int baudrate = this->declare_parameter("baudrate", 115200);
        std::string topic = this->declare_parameter("topic", "/imu/data");
        frame_id_ = this->declare_parameter("frame_id", "imu_link");
        
        // Đọc tần số public (nếu bạn muốn kiểm soát qua timer)
        double publish_rate = this->declare_parameter("publish_rate", 50.0);

        // 2. Khởi tạo Publisher bằng biến 'topic' vừa đọc được
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(topic, 10);
        
        // Topic euler tự động thêm đuôi "_euler" từ topic chính cho đồng bộ
        euler_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(topic + "_euler", 10);

        // 3. Khởi tạo sensor
        sensor_ = std::make_unique<hwt901::Hwt901Sensor>(port, baudrate);

        // 4. Gắn callback
        sensor_->set_data_callback([this](const hwt901::ImuData& data) {
            this->publish_imu_data(data);
            this->publish_euler_data(data);
        });

        sensor_->start();
        RCLCPP_INFO(this->get_logger(), "IMU Node started! Port: %s, Topic: %s, Frame: %s", 
                    port.c_str(), topic.c_str(), frame_id_.c_str());
    }

private:
    void publish_imu_data(const hwt901::ImuData& data) {
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->now();
        msg.header.frame_id = frame_id_; // Dùng biến frame_id_ lấy từ YAML

        msg.linear_acceleration.x = data.accel_x;
        msg.linear_acceleration.y = data.accel_y;
        msg.linear_acceleration.z = data.accel_z;

        msg.angular_velocity.x = data.gyro_x;
        msg.angular_velocity.y = data.gyro_y;
        msg.angular_velocity.z = data.gyro_z;

        tf2::Quaternion q;
        q.setRPY(data.roll, data.pitch, data.yaw);
        msg.orientation.x = q.x();
        msg.orientation.y = q.y();
        msg.orientation.z = q.z();
        msg.orientation.w = q.w();

        imu_pub_->publish(msg);
    }

    void publish_euler_data(const hwt901::ImuData& data) {
        auto euler_msg = geometry_msgs::msg::Vector3Stamped();
        euler_msg.header.stamp = this->now();
        euler_msg.header.frame_id = frame_id_; // Dùng biến frame_id_ lấy từ YAML

        euler_msg.vector.x = data.roll * 180 / M_PI;  
        euler_msg.vector.y = data.pitch * 180 / M_PI; 
        euler_msg.vector.z = data.yaw * 180 / M_PI;  

        euler_pub_->publish(euler_msg);
    }

    std::string frame_id_; // Biến lưu frame_id
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;
    std::unique_ptr<hwt901::Hwt901Sensor> sensor_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuPublisherNode>());
    rclcpp::shutdown();
    return 0;
}