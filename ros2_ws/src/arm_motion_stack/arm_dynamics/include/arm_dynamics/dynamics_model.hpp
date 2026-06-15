#pragma once

#include <string>

#include <Eigen/Dense>

namespace arm_dynamics
{

class DynamicsModel
{
public:
  bool loadModelFromUrdf(const std::string & urdf_path);
  Eigen::VectorXd computeGravity(const Eigen::VectorXd & q);
  Eigen::MatrixXd computeMassMatrix(const Eigen::VectorXd & q);
  Eigen::VectorXd computeInverseDynamics(
    const Eigen::VectorXd & q,
    const Eigen::VectorXd & dq,
    const Eigen::VectorXd & ddq);

private:
  std::string urdf_path_;
  int dof_{6};
};

}  // namespace arm_dynamics
