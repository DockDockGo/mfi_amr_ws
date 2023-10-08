#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "multi_target_tracker/track.hpp"
#include "foxglove_msgs/msg/scene_update.hpp"
#include "foxglove_msgs/msg/scene_entity.hpp"
#include "foxglove_msgs/msg/cube_primitive.hpp"
#include<vector>
#include<unordered_map>
#include<unordered_set>
#include<memory>

class MultiTargetTracker : public rclcpp::Node {
public:
    MultiTargetTracker(int n, int m, int k, double dt, unsigned int buffer_size) :
      Node("multi_target_tracker"),
      m_current_track_id(1),
      m_n(n),
      m_m(m),
      m_k(k),
      m_dt(dt),
      m_buffer_size(buffer_size) {
        m_detection_subscription = this->create_subscription<vision_msgs::msg::Detection3DArray>(
            "bounding_box",
            10,
            std::bind(&MultiTargetTracker::detection_cb, this, std::placeholders::_1)
        );
        m_predict_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MultiTargetTracker::predict_and_prune_timer_cb, this));
        m_pub_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MultiTargetTracker::pub_timer_cb, this));
        m_publisher_viz = this->create_publisher<foxglove_msgs::msg::SceneUpdate>("track_viz", 10);
    }
    void detection_cb(const vision_msgs::msg::Detection3DArray::SharedPtr msg);
    void associate_and_update_tracks(const std::vector<Eigen::MatrixXd>& measurements);
    void prune_tracks();
    void pub_timer_cb();
    void predict_and_prune_timer_cb();
    unsigned int get_associated_track(const Eigen::MatrixXd& measurement, const std::unordered_set<unsigned int>& invalid_tracks);
    double compute_iou(const Eigen::MatrixXd& box1, const Eigen::MatrixXd& box2);
    double compute_1d_inter(double c1, double l1, double c2, double l2);

private:
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr m_detection_subscription;
    rclcpp::TimerBase::SharedPtr m_pub_timer, m_predict_timer;
    rclcpp::Publisher<foxglove_msgs::msg::SceneUpdate>::SharedPtr m_publisher_viz;

    std::unordered_map<unsigned int, std::shared_ptr<Track>> m_tracks;
    unsigned int m_current_track_id;
    int m_n, m_m, m_k;
    double m_dt;
    static const double m_ASSOCIATION_DIST_THRESH, m_ASSOCIATION_IOU_THRESH, m_PRUNE_POS_STDDEV_THRESH, m_ASSOCIATION_SIZE_DIST_THRESH;
    unsigned int m_buffer_size;
};
