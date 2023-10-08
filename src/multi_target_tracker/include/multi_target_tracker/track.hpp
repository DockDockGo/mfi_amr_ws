#ifndef TRACK_HPP_
#define TRACK_HPP_

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <Eigen/LU>

class Track
{
public:
  Track(const unsigned int id, const int n, const int m, const int k, const double dt, const unsigned int buffer_size) :
    m_id(id),
    m_bel_mean(n, 1), // n x 1
    m_bel_cov(n, n),  // n x n
    m_A(n, n),        // n x n
    m_B(n, m),        // n x m
    m_C(k, n),        // k x n
    m_R_init(n, n),        // n x n
    m_Q_init(k, k),        // k x k
    m_dt(dt),
    m_start_time(0),
    m_prev_z(k, 1),
    m_buffer_size(buffer_size)
  {

    m_bel_cov << 1e-2,    0,    0,    0,    0,    0,    0,    0,    0,
                    0, 1e-2,    0,    0,    0,    0,    0,    0,    0,
                    0,    0, 1e-2,    0,    0,    0,    0,    0,    0,
                    0,    0,    0, 4e-2,    0,    0,    0,    0,    0,
                    0,    0,    0,    0, 4e-2,    0,    0,    0,    0,
                    0,    0,    0,    0,    0, 4e-2,    0,    0,    0,
                    0,    0,    0,    0,    0,    0, 1e-2,    0,    0,
                    0,    0,    0,    0,    0,    0,    0, 1e-2,    0,
                    0,    0,    0,    0,    0,    0,    0,    0, 1e-2;

    m_A       <<   1,    0,    0, m_dt,    0,    0,   0,   0,   0,
                   0,    1,    0,    0, m_dt,    0,   0,   0,   0,
                   0,    0,    1,    0,    0, m_dt,   0,   0,   0,
                   0,    0,    0,    1,    0,    0,   0,   0,   0,
                   0,    0,    0,    0,    1,    0,   0,   0,   0,
                   0,    0,    0,    0,    0,    1,   0,   0,   0,
                   0,    0,    0,    0,    0,    0,   1,   0,   0,
                   0,    0,    0,    0,    0,    0,   0,   1,   0,
                   0,    0,    0,    0,    0,    0,   0,   0,   1;

    m_B   = Eigen::MatrixXd::Zero(m_B.rows(), m_B.cols());               // n x m

    m_C   = Eigen::MatrixXd::Identity(m_C.rows(), m_C.cols());           // k x n

    m_Q_init  << 1e-2,    0,    0,    0,    0,    0,    0,    0,    0,
                    0, 1e-2,    0,    0,    0,    0,    0,    0,    0,
                    0,    0, 1e-2,    0,    0,    0,    0,    0,    0,
                    0,    0,    0, 4e-2,    0,    0,    0,    0,    0,
                    0,    0,    0,    0, 4e-2,    0,    0,    0,    0,
                    0,    0,    0,    0,    0, 4e-2,    0,    0,    0,
                    0,    0,    0,    0,    0,    0, 1e-2,    0,    0,
                    0,    0,    0,    0,    0,    0,    0, 1e-2,    0,
                    0,    0,    0,    0,    0,    0,    0,    0, 1e-2;

    m_Q = m_Q_init;

    m_R_init  << 1e-4,    0,    0,    0,    0,    0,     0,     0,     0,
                    0, 1e-4,    0,    0,    0,    0,     0,     0,     0,
                    0,    0, 1e-4,    0,    0,    0,     0,     0,     0,
                    0,    0,    0, 1e-3,    0,    0,     0,     0,     0,
                    0,    0,    0,    0, 1e-3,    0,     0,     0,     0,
                    0,    0,    0,    0,    0, 1e-3,     0,     0,     0,
                    0,    0,    0,    0,    0,    0, 5e-4,     0,     0,
                    0,    0,    0,    0,    0,    0,     0, 5e-4,     0,
                    0,    0,    0,    0,    0,    0,     0,     0, 5e-4;

    m_R = m_R_init;
  }
  ~Track() { std::cout << "------- Destructor called!" << std::endl; }
  void init(const Eigen::MatrixXd& bel_mean_init, rclcpp::Time ts) {
    m_bel_mean = bel_mean_init;
    m_prev_z = bel_mean_init;
    m_prev_meas_time = ts;

//    m_bel_mean_buffer.push_back(std::make_shared<Eigen::MatrixXd>(m_bel_mean.rows(), m_bel_mean.cols()));
//    *(m_bel_mean_buffer.back()) = m_bel_mean;
//    m_bel_cov_buffer.push_back(std::make_shared<Eigen::MatrixXd>(m_bel_cov.rows(), m_bel_cov.cols()));
//    *(m_bel_cov_buffer.back()) = m_bel_cov;
//    m_u_buffer.push_back(std::make_shared<Eigen::MatrixXd>(u.rows(), u.cols()));
//    *(m_u_buffer.back()) = u;

    update_cov();
  }

  void predict(const Eigen::MatrixXd& u);
  void update(const Eigen::MatrixXd& z, rclcpp::Time ts);
  void update_cov();
  double at_index(int index);
  Eigen::MatrixXd get_bel_mean();
  Eigen::MatrixXd get_pos_mean();
  Eigen::MatrixXd get_size_mean();
  Eigen::MatrixXd get_vel_mean();
  Eigen::MatrixXd get_prev_z();
  unsigned int get_id();
  double get_pos_cov();
  double get_delta_t_meas(rclcpp::Time ts_now);

private:
  Eigen::MatrixXd m_bel_mean; // n x 1
  Eigen::MatrixXd m_bel_cov;  // n x n
  Eigen::MatrixXd m_A;        // n x n
  Eigen::MatrixXd m_B;        // n x m
  Eigen::MatrixXd m_C;        // k x n
  Eigen::MatrixXd m_R_init;   // n x n
  Eigen::MatrixXd m_Q_init;   // k x k
  Eigen::MatrixXd m_R;        // n x n
  Eigen::MatrixXd m_Q;        // k x k
  Eigen::MatrixXd m_prev_z; // k x 1

  double m_dt;
  const unsigned int m_id;
  int m_start_time;
  rclcpp::Time m_prev_meas_time;
  unsigned int m_buffer_size;

  std::vector<std::shared_ptr<Eigen::MatrixXd>> m_bel_mean_buffer;
  std::vector<std::shared_ptr<Eigen::MatrixXd>> m_bel_cov_buffer;
  std::vector<std::shared_ptr<Eigen::MatrixXd>> m_u_buffer;
};

#endif  // TRACK_HPP_