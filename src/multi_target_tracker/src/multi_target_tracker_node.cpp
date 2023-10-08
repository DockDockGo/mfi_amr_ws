#include "multi_target_tracker/multi_target_tracker.hpp"


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiTargetTracker>(9, 3, 9, 0.1, 1);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

