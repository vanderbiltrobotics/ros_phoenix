#ifndef ROS_PHOENIX_BASE_COMPONENT
#define ROS_PHOENIX_BASE_COMPONENT

#include "ros_phoenix/phoenix_manager.hpp"

#include "ros_phoenix/msg/motor_control.hpp"
#include "ros_phoenix/msg/motor_status.hpp"

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"

#include <chrono>
#include <stdexcept>

using namespace rclcpp;
using namespace std::chrono_literals;

namespace ros_phoenix
{
    template <class MotorController, class Configuration, class FeedbackDevice, class ControlMode>
    class BaseComponent : public rclcpp::Node
    {
    public:
        explicit BaseComponent(const rclcpp::NodeOptions &options)
            : Node("motor", options)
        {
            // Detect if component run outside of a phoenix container
            if (!PhoenixManager::instanceCreated()) {
                RCLCPP_ERROR(this->get_logger(), "Phoenix components must be run inside phoenix container!");
                throw new std::runtime_error("Phoenix components must be run inside phoenix container!");
            }

            this->declare_parameter<int>("id", 0);
            this->declare_parameter<int>("period_ms", 20);
            this->declare_parameter<int>("watchdog_ms", 100);
            this->declare_parameter<int>("follow_id", -1);
            this->declare_parameter<int>("edges_per_rot", 4096); // Encoder edges per rotation (4096 is for built-in encoder)
            this->declare_parameter<bool>("invert", false);
            this->declare_parameter<bool>("invert_sensor", false);
            this->declare_parameter<bool>("brake_mode", true);
            this->declare_parameter<bool>("analog_input", false);
            this->declare_parameter<double>("max_voltage", 12);
            this->declare_parameter<double>("max_current", 30);
            this->declare_parameter<double>("sensor_multiplier", 1.0);

            this->declare_parameter<double>("P", 0);
            this->declare_parameter<double>("I", 0);
            this->declare_parameter<double>("D", 0);
            this->declare_parameter<double>("F", 0);

            this->period_ms_ = this->get_parameter("period_ms").as_int();
            this->watchdog_ms_ = this->get_parameter("watchdog_ms").as_int();
            this->follow_id_ = this->get_parameter("follow_id").as_int();

            this->controller_ = std::make_shared<MotorController>(this->get_parameter("id").as_int());

            std::string name(this->get_name());
            this->pub_ = this->create_publisher<ros_phoenix::msg::MotorStatus>(name + "/status", 1);
            this->sub_ = this->create_subscription<ros_phoenix::msg::MotorControl>(name + "/set", 1,
                                                                                   std::bind(&BaseComponent::set, this, std::placeholders::_1));

            this->reconfigure({});
            this->callback_handle_ = this->add_on_set_parameters_callback(std::bind(&BaseComponent::reconfigure, this, std::placeholders::_1));

            this->last_update_ = this->now();
            this->timer_ = this->create_wall_timer(
                std::chrono::milliseconds(this->period_ms_),
                std::bind(&BaseComponent::onTimer, this));
        }

        ~BaseComponent()
        {
            std::unique_lock<std::mutex> lock(this->config_mutex_);
            if (this->config_thread_)
            {
                this->configured_ = true; // Signal config thread to stop
                lock.unlock();
                this->config_thread_->join();
            }
        }

        void set(ros_phoenix::msg::MotorControl::UniquePtr msg)
        {
            this->last_update_ = this->now();
            this->watchdog_warned_ = false;

            if (this->configured_)
                this->controller_->Set(static_cast<ControlMode>(msg->mode), msg->value);
        }

    protected:
        rcl_interfaces::msg::SetParametersResult reconfigure(const std::vector<rclcpp::Parameter> &params)
        {
            std::lock_guard<std::mutex> guard(this->config_mutex_);

            for (auto param : params)
            {
                RCLCPP_INFO(this->get_logger(),
                            "Parameter changed: %s=%s",
                            param.get_name().c_str(),
                            param.value_to_string().c_str());
                if (param.get_name() == "id" || !this->controller_)
                {
                    this->controller_.reset();
                    this->controller_ = std::make_shared<MotorController>(param.as_int());
                }
                else if (param.get_name() == "period_ms" || !this->timer_)
                {
                    this->period_ms_ = param.as_int();
                    this->timer_.reset();
                    this->timer_ = this->create_wall_timer(
                        std::chrono::milliseconds(this->period_ms_),
                        std::bind(&BaseComponent::onTimer, this));
                }
                else if (param.get_name() == "watchdog_ms")
                {
                    this->watchdog_ms_ = param.as_int();
                }
            }

            if (!this->config_thread_)
            {
                this->configured_ = false;
                this->config_thread_ = std::make_shared<std::thread>(std::bind(&BaseComponent::configure, this));
            }
            else if (this->configured_)
            { // Thread needs to be joined and restarted
                this->config_thread_->join();
                this->configured_ = false;
                this->config_thread_ = std::make_shared<std::thread>(std::bind(&BaseComponent::configure, this));
            }

            auto result = rcl_interfaces::msg::SetParametersResult();
            result.successful = true;
            return result;
        }

        void configure()
        {
            bool warned = false;
            while (!this->configured_)
            {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                std::lock_guard<std::mutex> guard(this->config_mutex_);
                if (this->configured_)
                    break; // Break out if signaled to stop while sleeping

                if (this->controller_->GetFirmwareVersion() == -1)
                {
                    if (!warned)
                    {
                        RCLCPP_WARN(this->get_logger(), "Motor controller has not been seen and cannot be configured!");
                        warned = true;
                    }
                    continue;
                }
                SlotConfiguration slot;
                slot.kP = this->get_parameter("P").as_double();
                slot.kI = this->get_parameter("I").as_double();
                slot.kD = this->get_parameter("D").as_double();
                slot.kF = this->get_parameter("F").as_double();

                Configuration config;
                config.slot0 = slot;
                config.voltageCompSaturation = this->get_parameter("max_voltage").as_double();
                config.pulseWidthPeriod_EdgesPerRot = this->get_parameter("edges_per_rot").as_int();
                this->configure_current_limit(config);

                ErrorCode error = this->controller_->ConfigAllSettings(config, 50); // Takes up to 50ms
                if (error != ErrorCode::OK)
                {
                    if (!warned)
                    {
                        RCLCPP_WARN(this->get_logger(), "Motor controller configuration failed!");
                        warned = true;
                    }
                    continue;
                }

                if (this->get_parameter("brake_mode").as_bool())
                    this->controller_->SetNeutralMode(NeutralMode::Brake);
                else
                    this->controller_->SetNeutralMode(NeutralMode::Coast);

                this->configure_sensor();
                this->controller_->ConfigSelectedFeedbackCoefficient(this->get_parameter("sensor_multiplier").as_double());

                this->controller_->EnableVoltageCompensation(true);
                this->controller_->SetInverted(this->get_parameter("invert").as_bool());
                this->controller_->SetSensorPhase(this->get_parameter("invert_sensor").as_bool());
                this->controller_->SelectProfileSlot(0, 0);

                this->follow_id_ = this->get_parameter("follow_id").as_int();
                if (this->follow_id_ >= 0)
                    this->controller_->Set(ControlMode::Follower, this->follow_id_);

                RCLCPP_INFO(this->get_logger(), "Successfully configured Motor Controller");
                this->configured_ = true;
            }
        }

        void onTimer()
        {
            if (!this->configured_)
                return;

            if (this->follow_id_ < 0 && this->now() - this->last_update_ > rclcpp::Duration(this->watchdog_ms_ * 1000000))
            {
                this->controller_->Set(ControlMode::PercentOutput, 0.0);
                if (!this->watchdog_warned_)
                {
                    RCLCPP_WARN(this->get_logger(), "Watchdog timer expired: Motor ouput disabled");
                    this->watchdog_warned_ = true;
                }
            }

            ros_phoenix::msg::MotorStatus status;

            status.temperature = this->controller_->GetTemperature();
            status.bus_voltage = this->controller_->GetBusVoltage();

            status.output_percent = this->controller_->GetMotorOutputPercent();
            status.output_voltage = this->controller_->GetMotorOutputVoltage();
            status.output_current = this->get_output_current();

            status.position = this->controller_->GetSelectedSensorPosition();
            status.velocity = this->controller_->GetSelectedSensorVelocity();

            this->pub_->publish(status);
        }

        // Methods to be reimplemented by specific motor controllers with template specialization
        void configure_current_limit(Configuration &config __attribute__((unused))) {};
        void configure_sensor() {};
        double get_output_current() {
            return 0.0;
        };

    private:
        int period_ms_;
        int watchdog_ms_;
        bool watchdog_warned_ = true;
        int follow_id_;

        std::shared_ptr<MotorController> controller_;

        rclcpp::Time last_update_;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<ros_phoenix::msg::MotorStatus>::SharedPtr pub_;

        rclcpp::Subscription<ros_phoenix::msg::MotorControl>::SharedPtr sub_;

        OnSetParametersCallbackHandle::SharedPtr callback_handle_;

        //rclcpp::Service<tutorial_interfaces::srv::AddThreeInts>::SharedPtr reset_service_;

        bool configured_ = false;
        std::mutex config_mutex_;
        std::shared_ptr<std::thread> config_thread_;
    };

} // namespace ros_phoenix

#endif // ROS_PHOENIX_BASE_COMPONENT
