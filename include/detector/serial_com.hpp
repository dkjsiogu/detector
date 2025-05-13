#pragma once

#include <rclcpp/rclcpp.hpp>
#include <asio.hpp>
#include <asio/serial_port.hpp>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <asio/io_context.hpp>
#include <asio/serial_port.hpp>
#include <asio/steady_timer.hpp>

namespace serial_com {

class SerialPort {
public:
    using DataCallback = std::function<void(const std::vector<uint8_t>&)>;
    void stop();
    // 配置结构体（在类内部定义）
    struct Config {
        std::string port = "/dev/ttyUSB0";
        uint32_t baudrate = 115200;
        asio::serial_port::flow_control::type flow_control = 
            asio::serial_port::flow_control::none;
        asio::serial_port::parity::type parity = 
            asio::serial_port::parity::none;
        asio::serial_port::stop_bits::type stop_bits = 
            asio::serial_port::stop_bits::one;
        size_t timeout_ms = 1000;
    };

    // 数据包构建器
    class PacketBuilder {
    public:
        PacketBuilder& add(uint8_t byte) {
            packet_.push_back(byte);
            return *this;
        }

        PacketBuilder& add(const std::vector<uint8_t>& data) {
            packet_.insert(packet_.end(), data.begin(), data.end());
            return *this;
        }

        PacketBuilder& add(const std::string& str) {
            packet_.insert(packet_.end(), str.begin(), str.end());
            return *this;
        }

        template<typename T>
        PacketBuilder& add(const T& value) {
            const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
            packet_.insert(packet_.end(), bytes, bytes + sizeof(T));
            return *this;
        }

        std::vector<uint8_t> build() {
            return std::move(packet_);
        }

    private:
        std::vector<uint8_t> packet_;
    };

    SerialPort(rclcpp::Node& node, const Config& config);
    ~SerialPort();

    // 同步发送
    bool send(const std::vector<uint8_t>& data);
    
    // 异步发送（协程友好）
    void async_send(const std::vector<uint8_t>& data, 
        std::function<void(const asio::error_code&)> callback);

    // 设置数据接收回调
    void set_callback(DataCallback callback);

    // 获取数据包构建器
    PacketBuilder create_packet() {
        return PacketBuilder();
    }

    // 重新连接
    bool reconnect();

private:
    void start_async_read();
    void handle_read(const asio::error_code& ec, size_t bytes_transferred);

    rclcpp::Logger logger_;
    Config config_;
    asio::io_context io_context_;
    asio::serial_port serial_port_;
    asio::steady_timer timeout_timer_;
    std::unique_ptr<asio::io_context::work> work_;
    std::thread io_thread_;
    std::mutex mutex_;
    DataCallback callback_;
    std::vector<uint8_t> read_buffer_;
};

} // namespace serial_com