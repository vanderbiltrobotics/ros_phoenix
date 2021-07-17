#include "ros_phoenix/phoenix_manager.hpp"

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include <iostream>
#include <functional>

namespace ros_phoenix
{

    std::shared_ptr<PhoenixManager> PhoenixManager::singleton_;

    std::shared_ptr<PhoenixManager> PhoenixManager::createInstance(const std::string& interface, int period_ms, int watchdog_ms) {
        if (!PhoenixManager::singleton_) {
            PhoenixManager::singleton_ = std::shared_ptr<PhoenixManager>(new PhoenixManager(interface, period_ms, watchdog_ms));
        }
        return singleton_;
    }

    bool PhoenixManager::instanceCreated() {
        return bool(PhoenixManager::singleton_);
    }

    PhoenixManager::PhoenixManager(const std::string& interface, int period_ms, int watchdog_ms) {
        c_SetPhoenixDiagnosticsStartTime(1);

        ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

        this->watchdog_thread_ = std::thread(std::bind(&PhoenixManager::feedEnable, this, period_ms, watchdog_ms));
    }

    PhoenixManager::~PhoenixManager() {
        this->shutdown_requested_ = true;
        this->watchdog_thread_.join();
    }

    void PhoenixManager::feedEnable(int period_ms, int watchdog_ms) {
        while (!this->shutdown_requested_) {
            ctre::phoenix::unmanaged::Unmanaged::FeedEnable(watchdog_ms);
            std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
        }
    }

}