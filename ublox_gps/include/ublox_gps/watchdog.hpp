#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>
#include <iostream>

class Watchdog
{
public:
    explicit Watchdog(
        std::chrono::milliseconds timeout = std::chrono::milliseconds(1000),
        std::chrono::milliseconds check_interval = std::chrono::milliseconds(500),
        std::function<void()> callback = []() {} )
        : timeout_(timeout), check_interval_(check_interval), callback_(callback), enable_watchdog_(true), run_watchdog_(false),
          last_reset_(std::chrono::steady_clock::now())
    {
        watchdog_thread_ = std::thread([this]() {
            while (enable_watchdog_) {
                std::this_thread::sleep_for(check_interval_);

                if (!run_watchdog_) continue; 

                auto now = std::chrono::steady_clock::now();
                
                std::chrono::steady_clock::time_point last_reset_cp;
                {
                    std::lock_guard<std::mutex> lock(reset_mutex_);
                    last_reset_cp = last_reset_;
                }

                std::function<void()> current_callback;
                {
                    std::lock_guard<std::mutex> lock(callback_mutex_);
                    if ((now - last_reset_cp) >= timeout_) {
                        current_callback = callback_;
                    }
                    else {
                        current_callback = nullptr;
                    }
                }
                
                if (current_callback) {
                    current_callback();
                }
            }
        });
    }

    ~Watchdog(){
        enable_watchdog_ = false;
        if (watchdog_thread_.joinable()) {
            watchdog_thread_.join();
        }
    }

    void reset(){
        std::lock_guard<std::mutex> lock(reset_mutex_);
        last_reset_ = std::chrono::steady_clock::now();
    }

    void stop() noexcept {
        this->run_watchdog_ = false;
        std::cout << "[Watchdog] Stopped." << std::endl;
    }

    void start() {
        last_reset_ = std::chrono::steady_clock::now();
        this->run_watchdog_ = true;
        std::cout << "[Watchdog] Started with timeout: " << this->timeout_.count() << " ms" << std::endl;
    }

    void set_timeout(std::chrono::milliseconds timeout){
        timeout_ = timeout;
    }

    std::chrono::milliseconds get_timeout() const noexcept {
        return timeout_;
    }

    std::function<void()> get_callback() const noexcept{
        return callback_;
    }

    void set_callback(std::function<void()> callback) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        callback_ = std::move(callback);
    }
    void set_check_interval(std::chrono::milliseconds interval) {
        check_interval_ = interval;
    }

    std::chrono::milliseconds get_check_interval() const noexcept {
        return check_interval_;
    }
    
private:
    std::chrono::milliseconds timeout_;
    std::chrono::milliseconds check_interval_;
    std::function<void()> callback_;
    std::atomic<bool> enable_watchdog_;
    std::atomic<bool> run_watchdog_;
    std::mutex callback_mutex_;
    std::mutex reset_mutex_;
    std::chrono::steady_clock::time_point last_reset_;
    std::thread watchdog_thread_;
};