#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  std::vector<double> predicted_trajectory_xs;
  std::vector<double> predicted_trajectory_ys;
  std::vector<double> prev_actuations;

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  void set_latency (double latency);
  void set_velocity (double velocity);

 private:

  // delay before control actually applies in sec
  double latency_ = 0;
  // desired car velocity
  double ref_v_ = 60;

};

#endif /* MPC_H */
