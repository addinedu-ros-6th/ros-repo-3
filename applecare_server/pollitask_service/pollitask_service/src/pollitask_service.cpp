// move to src folder when running.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
// #include <geometry_msgs/msg/pose.hpp>

class PoseEstimator : public rclcpp::Node
{
public:
    PoseEstimator() : Node("pollitask_service")
    {
        // Publishers
        pollination_status_pub_ = this->create_publisher<std_msgs::msg::String>("pollitask_service", 10);

        // Subscribers
        applecare_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/applecare_service", 10, std::bind(&PoseEstimator::applecareCallback, this, std::placeholders::_1));
        arm_pollination_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/pollination_manager", 10, std::bind(&PoseEstimator::pollinationCallback, this, std::placeholders::_1));

        // Timer to publish pollination status
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PoseEstimator::publishPollinationTask, this));
    }

private:


    void applecareCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received from AppleCare Service (lsw): '%s'", msg->data.c_str());
    }

    void pollinationCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received from pollibot arm (leh): '%s'", msg->data.c_str());
    }

    void publishPollinationTask()
    {
        auto message = std_msgs::msg::String();
        message.data = "Pollination task in progress";
        RCLCPP_INFO(this->get_logger(), "Publishing PolliTask_Service (ksm): '%s'", message.data.c_str());
        pollination_status_pub_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pollination_status_pub_;
    // rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr applecare_sub_;
    // rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr arm_pollination_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr applecare_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_pollination_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseEstimator>());
    rclcpp::shutdown();
    return 0;
}
