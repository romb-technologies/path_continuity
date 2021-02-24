#ifndef TANGENT_ALIGNED
#define TANGENT_ALIGNED

#include "MotionModes/motion_mode.h"

namespace Romb
{
namespace MotionModes
{

class TangentAligned : public MotionMode
{
private:
  double angle_offset_;

public:
  TangentAligned(double angle_offset);

  MotionMode* clone() const override;

  double phi(const Bezier::Curve& curve, double t) const override;
  double phiDerived(const Bezier::Curve& curve, double t) const override;
  double phiDerived2(const Bezier::Curve& curve, double t) const override;
};

class Exponential : public MotionMode
{
private:
  double angle_offset_;
  double n_;

public:
  Exponential(double angle_offset, double n);

  MotionMode* clone() const override;

  double phi(const Bezier::Curve& curve, double t) const override;
  double phiDerived(const Bezier::Curve& curve, double t) const override;
  double phiDerived2(const Bezier::Curve& curve, double t) const override;
};

} // namespace ModesOfMotion
} // namespace Romb
#endif // TANGENT_ALIGNED
