#include <rclcpp/rclcpp.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>

#include <atomic>
#include <thread>
#include <mutex>
#include <vector>
#include <chrono>

class UDPTestReceiver : public rclcpp::Node
{
public:
    UDPTestReceiver()
        : Node("udp_test_receiver"),
          hs_sock_(-1), ss_sock_(-1),
          udp_running_(false),
          hs_frame_count_(0), ss_frame_count_(0),
          hs_frame_count_since_last_(0), ss_frame_count_since_last_(0),
          last_stats_time_(0.0)
    {
        this->declare_parameter<int>("hs_port", 9870);
        this->declare_parameter<int>("ss_port", 9871);
        this->declare_parameter<int>("hs_rows", 11);
        this->declare_parameter<int>("hs_cols", 17);

        int hs_port = this->get_parameter("hs_port").as_int();
        int ss_port = this->get_parameter("ss_port").as_int();
        hs_rows_ = this->get_parameter("hs_rows").as_int();
        hs_cols_ = this->get_parameter("hs_cols").as_int();

        RCLCPP_INFO(this->get_logger(),
            "UDP Test Receiver");
        RCLCPP_INFO(this->get_logger(),
            "  Height scan  -> port %d (%dx%d = %d floats, %d bytes)",
            hs_port, hs_rows_, hs_cols_,
            hs_rows_ * hs_cols_,
            hs_rows_ * hs_cols_ * sizeof(float));
        RCLCPP_INFO(this->get_logger(),
            "  Safety status -> port %d (1 byte)", ss_port);

        start_udp(hs_port, ss_port);

        stats_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() { print_stats(); });
    }

    ~UDPTestReceiver()
    {
        stop_udp();
    }

private:
    void start_udp(int hs_port, int ss_port)
    {
        udp_running_ = true;

        // Height scan socket
        hs_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (hs_sock_ >= 0)
        {
            int opt = 1;
            setsockopt(hs_sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

            struct sockaddr_in addr;
            std::memset(&addr, 0, sizeof(addr));
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = INADDR_ANY;
            addr.sin_port = htons(hs_port);

            if (bind(hs_sock_, (struct sockaddr*)&addr, sizeof(addr)) == 0)
            {
                struct timeval tv;
                tv.tv_sec = 1;
                tv.tv_usec = 0;
                setsockopt(hs_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

                hs_recv_thread_ = std::thread(&UDPTestReceiver::hs_udp_loop, this);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to bind height scan port %d", hs_port);
                close(hs_sock_);
                hs_sock_ = -1;
            }
        }

        // Safety status socket
        ss_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (ss_sock_ >= 0)
        {
            int opt = 1;
            setsockopt(ss_sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

            struct sockaddr_in addr;
            std::memset(&addr, 0, sizeof(addr));
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = INADDR_ANY;
            addr.sin_port = htons(ss_port);

            if (bind(ss_sock_, (struct sockaddr*)&addr, sizeof(addr)) == 0)
            {
                struct timeval tv;
                tv.tv_sec = 1;
                tv.tv_usec = 0;
                setsockopt(ss_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

                ss_recv_thread_ = std::thread(&UDPTestReceiver::ss_udp_loop, this);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to bind safety status port %d", ss_port);
                close(ss_sock_);
                ss_sock_ = -1;
            }
        }
    }

    void stop_udp()
    {
        udp_running_ = false;
        if (hs_recv_thread_.joinable()) hs_recv_thread_.join();
        if (ss_recv_thread_.joinable()) ss_recv_thread_.join();
        if (hs_sock_ >= 0) { close(hs_sock_); hs_sock_ = -1; }
        if (ss_sock_ >= 0) { close(ss_sock_); ss_sock_ = -1; }
    }

    void hs_udp_loop()
    {
        int expected_floats = hs_rows_ * hs_cols_;
        int expected_bytes = expected_floats * sizeof(float);
        std::vector<char> buf(expected_bytes);

        while (udp_running_)
        {
            ssize_t n = recv(hs_sock_, buf.data(), buf.size(), 0);
            if (n < 0) continue;

            if (n == expected_bytes)
            {
                hs_frame_count_++;
                hs_frame_count_since_last_++;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                    "Height scan: unexpected packet size %zd bytes (expected %d)",
                    n, expected_bytes);
            }
        }
    }

    void ss_udp_loop()
    {
        char buf[1];
        while (udp_running_)
        {
            ssize_t n = recv(ss_sock_, buf, sizeof(buf), 0);
            if (n < 0) continue;

            if (n == 1)
            {
                ss_frame_count_++;
                ss_frame_count_since_last_++;
            }
        }
    }

    void print_stats()
    {
        double now = this->now().seconds();
        double dt = now - last_stats_time_;

        if (dt > 0.0)
        {
            double hs_hz = hs_frame_count_since_last_ / dt;
            double ss_hz = ss_frame_count_since_last_ / dt;
            double mbps = (hs_frame_count_since_last_ * hs_rows_ * hs_cols_ * sizeof(float)) / (1024.0 * 1024.0 * dt);

            RCLCPP_INFO(this->get_logger(),
                "Height scan: %.1f Hz | %.3f MB/s | Total: %d  |  Safety: %.1f Hz | Total: %d",
                hs_hz, mbps, hs_frame_count_.load(),
                ss_hz, ss_frame_count_.load());
        }

        last_stats_time_ = now;
        hs_frame_count_since_last_ = 0;
        ss_frame_count_since_last_ = 0;
    }

    // Members
    int hs_sock_, ss_sock_;
    int hs_rows_, hs_cols_;
    std::atomic<bool> udp_running_;
    std::thread hs_recv_thread_, ss_recv_thread_;

    std::atomic<int> hs_frame_count_, ss_frame_count_;
    std::atomic<int> hs_frame_count_since_last_, ss_frame_count_since_last_;
    double last_stats_time_;

    rclcpp::TimerBase::SharedPtr stats_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UDPTestReceiver>());
    rclcpp::shutdown();
    return 0;
}
