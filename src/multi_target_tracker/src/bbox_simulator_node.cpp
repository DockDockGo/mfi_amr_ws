#include "rclcpp/rclcpp.hpp"

#include "vision_msgs/msg/bounding_box3_d.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

#include "foxglove_msgs/msg/scene_update.hpp"
#include "foxglove_msgs/msg/scene_entity.hpp"
#include "foxglove_msgs/msg/cube_primitive.hpp"

#include <vector>
#include <random>



class BboxSimulator : public rclcpp::Node {
public:
    BboxSimulator() : Node("bbox_simulator"), distribution_(0, 0.1), t_idx_(0) {
        publisher_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("bounding_box", 10);
        publisher_viz_ = this->create_publisher<foxglove_msgs::msg::SceneUpdate>("bounding_box_viz", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&BboxSimulator::timerCallback, this));

        cubes_no_noise_.push_back({0, 0, 0, 1, 1, 1, 0.1, 0, 0});
        cubes_no_noise_.push_back({0, 2, 0, 1, 1, 1, 0, 0, 0});

        cubes_ = cubes_no_noise_;

//        cubes_.push_back({0, 4, 0, 1, 1, 1, 0, 0, 0});
    }

private:
    void updateCubesVelocities() {
        for (auto& cube_no_noise : cubes_no_noise_){
            if (cube_no_noise[0] > 4){
                cube_no_noise[6] = -0.1;
            }
            else if (cube_no_noise[0] < -4){
                cube_no_noise[6] = 0.1;
            }
        }
    }

    void updateCubesPositions() {
        for (auto& cube_no_noise : cubes_no_noise_){
            cube_no_noise[0] += cube_no_noise[6];
            cube_no_noise[1] += cube_no_noise[7];
            cube_no_noise[2] += cube_no_noise[8];
        }

        for (int i = 0; i < cubes_.size(); i++){
            cubes_[i][0] = cubes_no_noise_[i][0] + distribution_(generator_);
            cubes_[i][1] = cubes_no_noise_[i][1] + distribution_(generator_);
            cubes_[i][2] = cubes_no_noise_[i][2] + distribution_(generator_);
        }
    }

    void timerCallback() {
        updateCubesVelocities();
        updateCubesPositions();
        if (t_idx_ % 5 == 0){
            publishBoundingBoxes();
            publishBoundingBoxesViz();
        }
        t_idx_++;
        if (t_idx_ > 50 && cubes_no_noise_.size() > 1) {
            cubes_no_noise_.erase(cubes_no_noise_.begin()+1);
            cubes_.erase(cubes_.begin()+1);
        }
    }

    void publishBoundingBoxes() {
        auto msg = std::make_unique<vision_msgs::msg::Detection3DArray>();
        for (auto cube : cubes_){
            vision_msgs::msg::Detection3D detection;
            detection.bbox.center.position.x = cube[0];
            detection.bbox.center.position.y = cube[1];
            detection.bbox.center.position.z = cube[2];
            detection.bbox.size.x = cube[3];
            detection.bbox.size.y = cube[4];
            detection.bbox.size.z = cube[5];
            msg->detections.push_back(detection);
        }
        publisher_->publish(std::move(msg));
    }

    void publishBoundingBoxesViz() {
        auto msg = std::make_unique<foxglove_msgs::msg::SceneUpdate>();
        foxglove_msgs::msg::SceneEntity scene_entity;
        scene_entity.frame_id = "velodyne";

        for (auto cube : cubes_) {
            foxglove_msgs::msg::CubePrimitive cube_primitive;
            cube_primitive.pose.position.x = cube[0];
            cube_primitive.pose.position.y = cube[1];
            cube_primitive.pose.position.z = cube[2];
            cube_primitive.size.x = cube[3];
            cube_primitive.size.y = cube[4];
            cube_primitive.size.z = cube[5];
            cube_primitive.color.g = 1;
            cube_primitive.color.a = 0.5;
            scene_entity.cubes.push_back(cube_primitive);
        }
        msg->entities.push_back(scene_entity);
        publisher_viz_->publish(std::move(msg));
    }

    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr publisher_;
    rclcpp::Publisher<foxglove_msgs::msg::SceneUpdate>::SharedPtr publisher_viz_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::vector<float>> cubes_, cubes_no_noise_;
    std::default_random_engine generator_;
    std::normal_distribution<double> distribution_;

    int t_idx_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BboxSimulator>());
    rclcpp::shutdown();
    return 0;
}