#ifndef MPC_H
#define MPC_H

#include <vector>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

// using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  double polyeval(Eigen::VectorXd coeffs, double x);
  Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
};

#endif /* MPC_H */
