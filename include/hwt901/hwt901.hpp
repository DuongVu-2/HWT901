#ifndef HWT901_HPP_
#define HWT901_HPP_

#include <boost/asio.hpp>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>

namespace hwt901 {

// Cấu trúc dữ liệu thuần C++ để không phụ thuộc vào ROS
struct ImuData {
    double accel_x, accel_y, accel_z;
    double gyro_x, gyro_y, gyro_z;
    double roll, pitch, yaw;
};

class Hwt901Sensor {
public:
    Hwt901Sensor(const std::string& port, int baudrate);
    ~Hwt901Sensor();

    // Đăng ký hàm callback. Hàm này sẽ được gọi khi có frame dữ liệu mới
    void set_data_callback(std::function<void(const ImuData&)> callback);

    void start();
    void stop();

private:
    void read_loop();
    bool check_sum(uint8_t type, const uint8_t* data, uint8_t sum);
    double convert_raw(uint8_t low, uint8_t high, double range);

    boost::asio::io_service io_service_;
    boost::asio::serial_port serial_;
    
    std::thread read_thread_;
    std::atomic<bool> running_;
    
    std::function<void(const ImuData&)> data_callback_;
    ImuData current_data_;
    std::mutex data_mutex_;
};

} // namespace hwt901

#endif // HWT901_SENSOR_HPP_