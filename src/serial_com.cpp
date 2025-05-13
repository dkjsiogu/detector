#include "detector/serial_com.hpp"

namespace serial_com
{

    SerialPort::SerialPort(rclcpp::Node &node, const Config &config)
        : logger_(node.get_logger()),
          config_(config),
          io_context_(),
          serial_port_(io_context_),
          timeout_timer_(io_context_),
          work_(std::make_unique<asio::io_context::work>(io_context_))
    {

        // 启动IO线程
        io_thread_ = std::thread([this]()
                                 { io_context_.run(); });

        if (!reconnect())
        {
            RCLCPP_ERROR(logger_, "Failed to initialize serial port");
        }
    }
    void SerialPort::stop()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // 停止异步读取操作
        if (serial_port_.is_open())
        {
            serial_port_.cancel(); // 取消所有异步操作
            serial_port_.close();  // 关闭串口
            RCLCPP_INFO(logger_, "Serial port %s closed successfully", config_.port.c_str());
        }
        else
        {
            RCLCPP_WARN(logger_, "Attempted to stop serial port, but it is already closed");
        }
    }
    SerialPort::~SerialPort()
    {
        work_.reset();
        if (io_thread_.joinable())
        {
            io_thread_.join();
        }
        if (serial_port_.is_open())
        {
            serial_port_.close();
        }
    }

    bool SerialPort::reconnect()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (serial_port_.is_open())
        {
            serial_port_.close();
        }

        try
        {
            serial_port_.open(config_.port);
            serial_port_.set_option(asio::serial_port::baud_rate(config_.baudrate));
            serial_port_.set_option(asio::serial_port::flow_control(config_.flow_control));
            serial_port_.set_option(asio::serial_port::parity(config_.parity));
            serial_port_.set_option(asio::serial_port::stop_bits(config_.stop_bits));

            start_async_read();
            RCLCPP_INFO(logger_, "Serial port %s opened successfully", config_.port.c_str());
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger_, "Serial port error: %s", e.what());
            return false;
        }
    }

    bool SerialPort::send(const std::vector<uint8_t> &data)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!serial_port_.is_open())
        {
            RCLCPP_WARN(logger_, "Attempted to send data while port is closed");
            return false;
        }

        try
        {
            size_t bytes_written = serial_port_.write_some(asio::buffer(data));
            if (bytes_written != data.size())
            {
                RCLCPP_WARN(logger_, "Partial write: %zu/%zu bytes", bytes_written, data.size());
                return false;
            }
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger_, "Write error: %s", e.what());
            return false;
        }
    }

    void SerialPort::async_send(const std::vector<uint8_t> &data,
                                std::function<void(const asio::error_code &)> callback)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!serial_port_.is_open())
        {
            RCLCPP_WARN(logger_, "Attempted async send while port is closed");
            callback(asio::error::make_error_code(asio::error::not_connected));
            return;
        }

        // 异步写入（回调风格）
        asio::async_write(
            serial_port_,
            asio::buffer(data),
            [this, cb = std::move(callback)](const asio::error_code &ec, size_t /*bytes*/)
            {
                if (ec)
                {
                    RCLCPP_ERROR(this->logger_, "Async write error: %s", ec.message().c_str());
                }
                cb(ec);
            });
    }
    void SerialPort::set_callback(DataCallback callback)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        callback_ = std::move(callback);
    }

    void SerialPort::start_async_read()
    {
        if (!serial_port_.is_open())
            return;

        read_buffer_.resize(1024);

        serial_port_.async_read_some(
            asio::buffer(read_buffer_),
            [this](const asio::error_code &ec, size_t bytes_transferred)
            {
                this->handle_read(ec, bytes_transferred);
            });
    }

    void SerialPort::handle_read(const asio::error_code &ec, size_t bytes_transferred)
    {
        if (ec)
        {
            if (ec == asio::error::operation_aborted)
            {
                RCLCPP_DEBUG(logger_, "Read operation aborted");
            }
            else
            {
                RCLCPP_ERROR(logger_, "Read error: %s", ec.message().c_str());
            }
            return;
        }

        if (bytes_transferred > 0)
        {
            std::vector<uint8_t> received_data(read_buffer_.begin(),
                                               read_buffer_.begin() + bytes_transferred);

            if (callback_)
            {
                callback_(received_data);
            }
        }

        // 继续下一次读取
        start_async_read();
    }

} // namespace serial_com