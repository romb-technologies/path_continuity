#include "MotionModes/motion_mode.h"

using namespace Romb;

MotionModes::MotionMode::~MotionMode() = default;

Eigen::Matrix2d MotionModes::MotionMode::Rot(const Bezier::Curve& curve, double t) const
{
  Eigen::Matrix2d rot;
  double phi_t = phi(curve, t);
  // clang-format off
  rot << std::cos(phi_t), -std::sin(phi_t),
         std::sin(phi_t),  std::cos(phi_t);
  // clang-format on
  return rot;
}

Eigen::Matrix2d MotionModes::MotionMode::RotDerived(const Bezier::Curve& curve, double t) const
{
  Eigen::Matrix2d rot_df;
  double phi_t = phi(curve, t);
  double phi_dt = phiDerived(curve, t);
  // clang-format off
  rot_df << -std::sin(phi_t), -std::cos(phi_t),
             std::cos(phi_t), -std::sin(phi_t);
  // clang-format on
  return rot_df * phi_dt;
}

Eigen::Matrix2d MotionModes::MotionMode::RotDerived2(const Bezier::Curve& curve, double t) const
{
  Eigen::Matrix2d rot_df;
  Eigen::Matrix2d rot_df2;
  double phi_t = phi(curve, t);
  double phi_dt = phiDerived(curve, t);
  double phi_dt2 = phiDerived2(curve, t);
  // clang-format off
  rot_df << -std::sin(phi_t), -std::cos(phi_t),
             std::cos(phi_t), -std::sin(phi_t);
  rot_df2 << -std::cos(phi_t),  std::sin(phi_t),
             -std::sin(phi_t), -std::cos(phi_t);
  // clang-format on
  return rot_df2 * phi_dt * phi_dt + rot_df * phi_dt2;
}
