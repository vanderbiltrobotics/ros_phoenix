#include "ros_phoenix/base_node.hpp"
#include "ros_phoenix/phoenix_manager.hpp"

namespace ros_phoenix {

const std::string BaseNode::Parameter::ID = "id";

BaseNode::BaseNode(const std::string& name, const rclcpp::NodeOptions& options)
    : rclcpp::Node(name, options)
{
    // Detect if component run outside of a phoenix container
    if (!PhoenixManager::instanceCreated()) {
        RCLCPP_ERROR(
            this->get_logger(), "Phoenix components must be run inside phoenix container!");
        throw new std::runtime_error("Phoenix components must be run inside phoenix container!");
    }

    this->declare_parameter<int>(Parameter::ID, 0);
    this->declare_parameter<int>("watchdog_ms", 100);
    this->template declare_parameter<int>("period_ms", 20);
    this->declare_parameter<int>("follow_id", -1);
    this->declare_parameter<int>(
        "edges_per_rot", 4096); // Encoder edges per rotation (4096 is for built-in encoder)
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

    this->watchdog_ms_ = this->get_parameter("watchdog_ms").as_int();
    this->period_ms_ = this->get_parameter("period_ms").as_int();
    this->follow_id_ = this->get_parameter("follow_id").as_int();
    this->sensor_multiplier_ = this->get_parameter("sensor_multiplier").as_double();

    this->last_update_ = this->now();

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(this->get_parameter("period_ms").as_int()),
        std::bind(&BaseNode::onTimer, this));
}

BaseNode::~BaseNode()
{
    std::unique_lock<std::mutex> lock(this->config_mutex_);
    if (this->config_thread_) {
        this->configured_ = true; // Signal config thread to stop
        lock.unlock();
        this->config_thread_->join();
    }
}

void BaseNode::initialize()
{
    this->set_parameters_callback_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) { return this->reconfigure(params); });

    this->reconfigure({});
}

void BaseNode::set(MotorControl::SharedPtr control_msg __attribute__((unused)))
{
    this->last_update_ = this->now();
    this->watchdog_warned_ = false;
}

rcl_interfaces::msg::SetParametersResult BaseNode::reconfigure(
    const std::vector<rclcpp::Parameter>& params)
{
    RCLCPP_INFO(this->get_logger(), "phoenix_node reconfigure()");
    for (auto param : params) {
        RCLCPP_DEBUG(this->get_logger(), "Parameter changed: %s=%s", param.get_name().c_str(),
            param.value_to_string().c_str());
        if (param.get_name() == "watchdog_ms") {
            this->watchdog_ms_ = param.as_int();

        } else if (param.get_name() == "period_ms") {
            this->timer_ = this->create_wall_timer(
                std::chrono::milliseconds(this->get_parameter("period_ms").as_int()),
                std::bind(&BaseNode::onTimer, this));

        } else if (param.get_name() == "follow_id") {
            this->follow_id_ = this->get_parameter("follow_id").as_int();

        } else if (param.get_name() == "sensor_multiplier") {
            this->sensor_multiplier_ = this->get_parameter("sensor_multiplier").as_double();
        }
    }

    if (!this->config_thread_) {
        this->configured_ = false;
        this->config_thread_ = std::make_shared<std::thread>(std::bind(&BaseNode::configure, this));
    } else if (this->configured_) { // Thread needs to be joined and restarted
        this->config_thread_->join();
        this->configured_ = false;
        this->config_thread_ = std::make_shared<std::thread>(std::bind(&BaseNode::configure, this));
    }

    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    return result;
}

void BaseNode::onTimer()
{
    if (!this->configured_)
        return;

    auto last_update = this->last_update_;
    if (this->follow_id_ < 0
        && this->now() - last_update > std::chrono::milliseconds(this->watchdog_ms_)) {
        MotorControl::SharedPtr msg = std::make_shared<MotorControl>();
        msg->mode = MotorControl::PERCENT_OUTPUT;
        msg->value = 0.0;
        this->set(msg);
        this->last_update_ = last_update; // Reset change to value caused by this->set()

        if (!this->watchdog_warned_) {
            RCLCPP_WARN(this->get_logger(), "Watchdog timer expired: Motor ouput disabled");
            this->watchdog_warned_ = true;
        }
    }
}

} // namespace ros_phoenix