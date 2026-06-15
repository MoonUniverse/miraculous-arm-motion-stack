#include "arm_dynamics/dynamics_model.hpp"

#include <utility>

namespace arm_dynamics
{

bool DynamicsModel::loadModelFromUrdf(const std::string & urdf_path)
{
  urdf_path_ = urdf_path;
  dof_ = 6;
  return !urdf_path_.empty();
}

Eigen::VectorXd DynamicsModel::computeGravity(const Eigen::VectorXd & q)
{
  return Eigen::VectorXd::Zero(q.size());
}

Eigen::MatrixXd DynamicsModel::computeMassMatrix(const Eigen::VectorXd & q)
{
  return Eigen::MatrixXd::Identity(q.size(), q.size());
}

Eigen::VectorXd DynamicsModel::computeInverseDynamics(
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & dq,
  const Eigen::VectorXd & ddq)
{
  (void)dq;
  if (ddq.size() == q.size()) {
    return ddq;
  }
  return Eigen::VectorXd::Zero(q.size());
}

}  // namespace arm_dynamics
