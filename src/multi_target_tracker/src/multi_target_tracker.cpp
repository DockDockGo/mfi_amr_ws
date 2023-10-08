#include "multi_target_tracker/multi_target_tracker.hpp"

const double MultiTargetTracker::m_ASSOCIATION_DIST_THRESH = 1;
const double MultiTargetTracker::m_ASSOCIATION_SIZE_DIST_THRESH = 1;
const double MultiTargetTracker::m_ASSOCIATION_IOU_THRESH = 0.2;
const double MultiTargetTracker::m_PRUNE_POS_STDDEV_THRESH = 1;


void MultiTargetTracker::detection_cb(const vision_msgs::msg::Detection3DArray::SharedPtr msg) {
    std::vector<Eigen::MatrixXd> measurements;
    for (const auto& detection : msg->detections) {
        double x = detection.bbox.center.position.x;
        double y = detection.bbox.center.position.y;
        double z = detection.bbox.center.position.z;
        Eigen::MatrixXd measurement(m_k, 1);
        measurement << detection.bbox.center.position.x, detection.bbox.center.position.y, detection.bbox.center.position.z,
                       0, 0, 0, detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z;
        measurements.push_back(measurement);
    }
    associate_and_update_tracks(measurements);
}

unsigned int MultiTargetTracker::get_associated_track(const Eigen::MatrixXd& measurement, const std::unordered_set<unsigned int>& invalid_tracks) {
//    std::cout << "---------------------------- num tracks: " << m_tracks.size() << std::endl;
    unsigned int associated_track = 0;
    double highest_iou = 0;
    for (auto track : m_tracks) {
        double iou = compute_iou(measurement, track.second->get_bel_mean());
//        if (iou > m_ASSOCIATION_IOU_THRESH && iou > highest_iou && invalid_tracks.count(track.first) == 0){
        if (iou > m_ASSOCIATION_IOU_THRESH && iou > highest_iou){
            associated_track = track.second->get_id();
            highest_iou = iou;
        }
    }
    if (associated_track != 0){
        return associated_track;
    }

    for (auto track : m_tracks) {
        double dist_centers = (measurement.block(0, 0, 3, 1) - track.second->get_pos_mean()).norm();
        double dist_size = (measurement.block(6, 0, 3, 1) - track.second->get_size_mean()).norm();
//        if (dist_centers < m_ASSOCIATION_DIST_THRESH && dist_size < m_ASSOCIATION_SIZE_DIST_THRESH){
        if (dist_centers < m_ASSOCIATION_DIST_THRESH && dist_size < m_ASSOCIATION_SIZE_DIST_THRESH && invalid_tracks.count(track.first) == 0){
            associated_track = track.second->get_id();
        }
    }

    return associated_track;
}

void MultiTargetTracker::associate_and_update_tracks(const std::vector<Eigen::MatrixXd>& measurements){
    std::unordered_set<unsigned int> associated_tracks;

    for (auto measurement : measurements){
        std::cout << "Received new measurement " << measurement.transpose() << std::endl;
        unsigned int associated_track_id = get_associated_track(measurement, associated_tracks);
        if (associated_track_id == 0){
            std::cout << "No association found! Creating new track!" << std::endl;
            m_tracks[m_current_track_id] = std::make_shared<Track>(m_current_track_id, m_n, m_m, m_k, m_dt, m_buffer_size);
            m_tracks[m_current_track_id]->init(measurement, this->now());
            associated_tracks.insert(m_current_track_id);
            m_current_track_id++;
        }
        else {
            Eigen::MatrixXd prev_measurement = m_tracks[associated_track_id]->get_prev_z();
            double delta_t_meas = m_tracks[associated_track_id]->get_delta_t_meas(this->now());
            if (delta_t_meas > 0.01){
                if (measurement(0, 0) - prev_measurement(0, 0) > 0.1){
                    measurement(3, 0) = 0.3 * m_tracks[associated_track_id]->at_index(3) + 0.7 * (measurement(0, 0) - prev_measurement(0, 0)) / delta_t_meas;

                }
                if (measurement(0, 0) - prev_measurement(0, 0) > 0.1){
                    measurement(4, 0) = 0.3 * m_tracks[associated_track_id]->at_index(4) + 0.7 * (measurement(1, 0) - prev_measurement(1, 0)) / delta_t_meas;
                }
                if (measurement(0, 0) - prev_measurement(0, 0) > 0.1){
                    measurement(5, 0) = 0.3 * m_tracks[associated_track_id]->at_index(5) + 0.7 * (measurement(2, 0) - prev_measurement(2, 0)) / delta_t_meas;
                }
            }
            m_tracks[associated_track_id]->update(measurement, this->now());
            associated_tracks.insert(associated_track_id);
        }
    }
}

void MultiTargetTracker::prune_tracks(void) {
    std::vector<unsigned int> ids_to_prune;
    for (auto track : m_tracks) {
        if (sqrt(track.second->get_pos_cov()) > m_PRUNE_POS_STDDEV_THRESH){
            ids_to_prune.push_back(track.first);
            std::cout << "Pruning track id " << track.first << " because of high pos stddev (" << sqrt(track.second->get_pos_cov()) << ")!" << std::endl;
        }
    }
    for (auto id : ids_to_prune) {
        m_tracks.erase(id);
    }
}

void MultiTargetTracker::predict_and_prune_timer_cb() {
    for (auto track : m_tracks) {
        track.second->predict(Eigen::MatrixXd::Zero(m_m, 1));
    }
    prune_tracks();
}

void MultiTargetTracker::pub_timer_cb() {

    auto msg = std::make_unique<foxglove_msgs::msg::SceneUpdate>();
    foxglove_msgs::msg::SceneEntity scene_entity;
    scene_entity.frame_id = "velodyne";
    for (auto track : m_tracks) {
        foxglove_msgs::msg::CubePrimitive cube_primitive;
        cube_primitive.pose.position.x = track.second->at_index(0);
        cube_primitive.pose.position.y = track.second->at_index(1);
        cube_primitive.pose.position.z = track.second->at_index(2);
        cube_primitive.size.x = track.second->at_index(6);
        cube_primitive.size.y = track.second->at_index(7);
        cube_primitive.size.z = track.second->at_index(8);
        cube_primitive.color.b = 1;
        cube_primitive.color.a = 0.5;
        scene_entity.cubes.push_back(cube_primitive);
    }
    msg->entities.push_back(scene_entity);
    m_publisher_viz->publish(std::move(msg));
}

double MultiTargetTracker::compute_1d_inter(double c1, double l1, double c2, double l2) {
    double left  = std::max(c1 - l1 / 2.0, c2 - l2 / 2.0);
    double right = std::min(c1 + l1 / 2.0, c2 + l2 / 2.0);
    return std::max(0.0, right - left);
}

double MultiTargetTracker::compute_iou(const Eigen::MatrixXd& box1, const Eigen::MatrixXd& box2) {
    double x_int = compute_1d_inter(box1(0, 0), box1(6, 0), box2(0, 0), box2(6, 0));
    double y_int = compute_1d_inter(box1(1, 0), box1(7, 0), box2(1, 0), box2(7, 0));
    double z_int = compute_1d_inter(box1(2, 0), box1(8, 0), box2(2, 0), box2(8, 0));
    double intersection_bboxes = x_int * y_int * z_int;
    double union_bboxes = box1(6, 0) * box1(7, 0) * box1(8, 0) + box2(6, 0) * box2(7, 0) * box2(8, 0) - intersection_bboxes;
    return intersection_bboxes / union_bboxes;
}