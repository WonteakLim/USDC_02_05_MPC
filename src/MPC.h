#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

// =============================
// Weight
const double Wcte_ = 200.0;
const double Weps_ = 50.0;
const double Wspd_ = 0.3;
const double Wdel_ = 1.0;
const double Wacc_ = 1.0;
const double Wdde_ = 50.0;
const double Wdac_ = 10.0;
const double Wcur_ = 100.0;

// =============================
// Configuration
const double max_a_ = 5.0;
const double max_v_ = 50.0;

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  vector<double> solution_;
  vector<double> getTrajectory(void) { return solution_; }

};

#endif /* MPC_H */
