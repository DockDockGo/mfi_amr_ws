#include "multi_target_tracker/track.hpp"

void Track::predict(const Eigen::MatrixXd& u) {
    m_bel_mean = m_A * m_bel_mean + m_B * u;
    m_bel_cov = m_A * m_bel_cov * m_A.transpose() + m_R;

    while (m_bel_mean_buffer.size() >= m_buffer_size) {
        m_bel_mean_buffer.erase(m_bel_mean_buffer.begin());
        m_bel_cov_buffer.erase(m_bel_cov_buffer.begin());
        m_u_buffer.erase(m_u_buffer.begin());
    }

    m_bel_mean_buffer.push_back(std::make_shared<Eigen::MatrixXd>(m_bel_mean.rows(), m_bel_mean.cols()));
    *(m_bel_mean_buffer.back()) = m_bel_mean;

    m_bel_cov_buffer.push_back(std::make_shared<Eigen::MatrixXd>(m_bel_cov.rows(), m_bel_cov.cols()));
    *(m_bel_cov_buffer.back()) = m_bel_cov;

    m_u_buffer.push_back(std::make_shared<Eigen::MatrixXd>(u.rows(), u.cols()));
    *(m_u_buffer.back()) = u;

    update_cov();
}

void Track::update(const Eigen::MatrixXd& z, rclcpp::Time ts) {
   if (m_bel_mean_buffer.size() == 0){
      Eigen::MatrixXd K = m_bel_cov * m_C.transpose() * (m_C * m_bel_cov * m_C.transpose() + m_Q).inverse();
      m_bel_mean = m_bel_mean + K * (z - m_C * m_bel_mean);
      m_bel_cov = (Eigen::MatrixXd::Identity(m_bel_cov.rows(), m_bel_cov.cols()) - K * m_C) * m_bel_cov;
   }
   else {
       Eigen::MatrixXd bel_mean_temp(m_bel_mean.rows(), m_bel_mean.cols()), bel_cov_temp(m_bel_cov.rows(), m_bel_cov.cols()), u_temp(9, 1);

       Eigen::MatrixXd K = *(m_bel_cov_buffer.front()) * m_C.transpose() * (m_C * *(m_bel_cov_buffer.front()) * m_C.transpose() + m_Q).inverse();
       *(m_bel_mean_buffer.front()) = *(m_bel_mean_buffer.front()) + K * (z - m_C * *(m_bel_mean_buffer.front()));
       *(m_bel_cov_buffer.front()) = (Eigen::MatrixXd::Identity(m_bel_cov.rows(), m_bel_cov.cols()) - K * m_C) * *(m_bel_cov_buffer.front());

       for (int i = 1; i < m_bel_mean_buffer.size(); i++){
           (*(m_bel_mean_buffer[i])) = m_A * (*(m_bel_mean_buffer[i])) + m_B * (*(m_u_buffer[i]));
           (*(m_bel_cov_buffer[i])) = m_A * (*(m_bel_cov_buffer[i])) * m_A.transpose() + m_R;
       }
       m_bel_mean = *(m_bel_mean_buffer.back());
       m_bel_cov = *(m_bel_cov_buffer.back());
   }

   update_cov();
   m_prev_z = z;
   m_prev_meas_time = ts;
}

void Track::update_cov() {
//    m_R(0, 0) = m_R_init(0, 0) + 2.0 * abs(m_bel_mean(3, 0));
//    m_R(1, 1) = m_R_init(1, 1) + 2.0 * abs(m_bel_mean(4, 0));
//    m_R(2, 2) = m_R_init(2, 2) + 2.0 * abs(m_bel_mean(5, 0));
//
//    m_R(3, 3) = m_R_init(3, 3) + 2.0 * abs(m_bel_mean(3, 0));
//    m_R(4, 4) = m_R_init(4, 4) + 2.0 * abs(m_bel_mean(4, 0));
//    m_R(5, 5) = m_R_init(5, 5) + 2.0 * abs(m_bel_mean(5, 0));

//    std::cout << "m_R " << m_R(0, 0) << ", " << m_R(1, 1) << ", " << m_R(2, 2) << std::endl;
}

double Track::at_index(int index) {
   return m_bel_mean(index, 0);
}

Eigen::MatrixXd Track::get_bel_mean() {
   return m_bel_mean;
}

Eigen::MatrixXd Track::get_pos_mean() {
   return m_bel_mean.block(0, 0, 3, 1);
}

Eigen::MatrixXd Track::get_vel_mean() {
   return m_bel_mean.block(3, 0, 3, 1);
}

Eigen::MatrixXd Track::get_size_mean() {
   return m_bel_mean.block(6, 0, 3, 1);
}

Eigen::MatrixXd Track::get_prev_z() {
   return m_prev_z;
}

double Track::get_delta_t_meas(rclcpp::Time ts_now){
    return (ts_now - m_prev_meas_time).seconds();
}

unsigned int Track::get_id() {
    return m_id;
}

double Track::get_pos_cov() {
    return m_bel_cov(0, 0) + m_bel_cov(1, 1) + m_bel_cov(2, 2);
}
