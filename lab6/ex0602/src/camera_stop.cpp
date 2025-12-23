#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>

class DepthStop : public rclcpp::Node {
public:
    DepthStop() : Node("depth_stop") {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/depth_camera/depth_image", 10,
            std::bind(&DepthStop::depth_callback, this, std::placeholders::_1));
    }

private:
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Данные глубины в Gazebo обычно передаются как float32 (4 байта на пиксель)
        const float* depth_data = reinterpret_cast<const float*>(&msg->data[0]);
        int width = msg->width;
        int height = msg->height;

        // Проверяем центральную область (например, квадрат 20x20 в центре)
        bool obstacle = false;
        for (int y = height/2 - 10; y < height/2 + 10; ++y) {
            for (int x = width/2 - 10; x < width/2 + 10; ++x) {
                float dist = depth_data[y * width + x];
                if (dist > 0.1 && dist < 0.5) { // 0.5 метра
                    obstacle = true;
                    break;
                }
            }
        }

        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = obstacle ? 0.0 : 0.3;
        pub_->publish(cmd);
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthStop>());
    rclcpp::shutdown();
    return 0;
}
