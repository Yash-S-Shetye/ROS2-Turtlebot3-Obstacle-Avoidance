#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/qos.hpp"

constexpr double distToObstacle = 1.0;

class ObstacleAvoidance : public rclcpp::Node
{
public:
    ObstacleAvoidance()
        : Node("obstacle_avoidance")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            qos,
            std::bind(&ObstacleAvoidance::obstacle_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void obstacle_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "The distance to obstacle is %f ", msg->ranges[300]);

        auto move = geometry_msgs::msg::Twist();

        if (msg->ranges[300] > distToObstacle)
        {
            move.linear.x = 0.5;
            move.angular.z = 0.0;
        }

        if (msg->ranges[300] <= distToObstacle)
        {
            move.linear.x = 0.5;
            move.angular.z = 3.5;
        }

        publisher_->publish(move);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto obstacle_avoidance = std::make_shared<ObstacleAvoidance>();

    rclcpp::spin(obstacle_avoidance);

    rclcpp::shutdown();

    return 0;
}

