#ifndef ROMB_MOTION_MODE_H
#define ROMB_MOTION_MODE_H

#include <Bezier/declarations.h>

//#include "AgvDynamics/geometry.h"

namespace Romb
{
namespace MotionModes
{

inline double wrapToPi(double angle){
  while (angle > M_PI)
    angle -= 2*M_PI;
  while (angle < -M_PI)
    angle += 2*M_PI;
  return angle;
}

class MotionMode
{
public:
  std::string type;
  MotionMode() = default;
  virtual ~MotionMode();
  MotionMode(const MotionMode&) = delete;
  MotionMode(MotionMode&&) = delete;
  auto operator=(const MotionMode&) = delete;
  auto operator=(MotionMode&&) = delete;

  virtual MotionMode* clone() const = 0;

  virtual double phi(const Bezier::Curve& curve, double t) const = 0;
  virtual double phiDerived(const Bezier::Curve& curve, double t) const = 0;
  virtual double phiDerived2(const Bezier::Curve& curve, double t) const = 0;

  Eigen::Matrix2d Rot(const Bezier::Curve& curve, double t) const;
  Eigen::Matrix2d RotDerived(const Bezier::Curve& curve, double t) const;
  Eigen::Matrix2d RotDerived2(const Bezier::Curve& curve, double t) const;
};

} // namespace ModesOfMotion
} // namespace Romb
#endif // ROMB_MOTION_MODE_H
