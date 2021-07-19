#ifndef ROS_PHOENIX_PHOENIX_MANAGER
#define ROS_PHOENIX_PHOENIX_MANAGER

#include <memory>
#include <string>
#include <thread>

namespace ros_phoenix
{

    class PhoenixManager {
    public:
        static std::shared_ptr<PhoenixManager> createInstance(const std::string& interface, int period_ms, int watchdog_ms);

        static bool instanceCreated();

        ~PhoenixManager();

    private:
        static std::shared_ptr<PhoenixManager> singleton_;

        PhoenixManager(const std::string& interface, int period_ms, int watchdog_ms);

        void feedEnable(int period_ms, int watchdog_ms);

        bool shutdown_requested_ = false;
        std::thread watchdog_thread_;

    };

}

#endif // ROS_PHOENIX_PHOENIX_MANAGER