#include "hwt901/hwt901.hpp"
#include <iostream>
#include <cmath>

namespace hwt901 {

Hwt901Sensor::Hwt901Sensor(const std::string& port, int baudrate) 
    : serial_(io_service_), running_(false) {
    try {
        serial_.open(port);
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
        serial_.set_option(boost::asio::serial_port_base::character_size(8));
        serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    } catch (const std::exception& e) {
        std::cerr << "Lỗi mở cổng Serial: " << e.what() << std::endl;
    }
}

Hwt901Sensor::~Hwt901Sensor() {
    stop();
}

void Hwt901Sensor::set_data_callback(std::function<void(const ImuData&)> callback) {
    data_callback_ = callback;
}

void Hwt901Sensor::start() {
    if (running_) return;
    running_ = true;
    read_thread_ = std::thread(&Hwt901Sensor::read_loop, this);
}

void Hwt901Sensor::stop() {
    running_ = false;
    if (serial_.is_open()) serial_.close();
    if (read_thread_.joinable()) read_thread_.join();
}

double Hwt901Sensor::convert_raw(uint8_t low, uint8_t high, double range) {
    int16_t raw = (static_cast<int16_t>(high) << 8) | low;
    return (static_cast<double>(raw) / 32768.0) * range;
}

bool Hwt901Sensor::check_sum(uint8_t type, const uint8_t* data, uint8_t sum) {
    uint8_t calculated = 0x55 + type;
    for (int i = 1; i < 9; ++i) {
        calculated += data[i];
    }
    return calculated == sum;
}

void Hwt901Sensor::read_loop() {
    const double G_TO_MS2 = 9.80665;
    const double DEG_TO_RAD = M_PI / 180.0;
    uint8_t head;

    while (running_ && serial_.is_open()) {
        try {
            boost::asio::read(serial_, boost::asio::buffer(&head, 1));
            if (head == 0x55) {
                uint8_t payload[10];
                boost::asio::read(serial_, boost::asio::buffer(payload, 10));

                uint8_t type = payload[0];
                
                // Bỏ qua nếu sai checksum
                if (!check_sum(type, payload, payload[9])) continue;

                std::lock_guard<std::mutex> lock(data_mutex_);
                bool data_updated = false;

                if (type == 0x51) { // Gia tốc (16g) [cite: 139]
                    current_data_.accel_x = convert_raw(payload[1], payload[2], 16.0) * G_TO_MS2;
                    current_data_.accel_y = convert_raw(payload[3], payload[4], 16.0) * G_TO_MS2;
                    current_data_.accel_z = convert_raw(payload[5], payload[6], 16.0) * G_TO_MS2;
                } 
                else if (type == 0x52) { // Vận tốc góc (2000 deg/s) [cite: 155]
                    current_data_.gyro_x = convert_raw(payload[1], payload[2], 2000.0) * DEG_TO_RAD;
                    current_data_.gyro_y = convert_raw(payload[3], payload[4], 2000.0) * DEG_TO_RAD;
                    current_data_.gyro_z = convert_raw(payload[5], payload[6], 2000.0) * DEG_TO_RAD;
                }
                else if (type == 0x53) { // Góc thái độ (180 deg) [cite: 168]
                    current_data_.roll = convert_raw(payload[1], payload[2], 180.0) * DEG_TO_RAD;
                    current_data_.pitch = convert_raw(payload[3], payload[4], 180.0) * DEG_TO_RAD;
                    current_data_.yaw = convert_raw(payload[5], payload[6], 180.0) * DEG_TO_RAD;
                    
                    data_updated = true; // Thường gói 0x53 là gói cuối trong chu trình xuất, nên trigger callback ở đây
                }

                if (data_updated && data_callback_) {
                    data_callback_(current_data_);
                }
            }
        } catch (const boost::system::system_error& e) {
            if (running_) std::cerr << "Lỗi đọc Serial: " << e.what() << std::endl;
        }
    }
}

} // namespace hwt901