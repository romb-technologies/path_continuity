#include "MotionModes/crab.h"

#define UNUSED(x) (void)(x);

using namespace Romb;
using namespace MotionModes;

Crab::Crab(double phi) : phi_(phi) {type = "crab";}

MotionMode* Crab::clone() const { return new Crab(phi_); }

double Crab::phi(const Bezier::Curve& curve, double t) const
{
  UNUSED(curve)
  UNUSED(t)
  return phi_;
}

double Crab::phiDerived(const Bezier::Curve& curve, double t) const
{
  UNUSED(curve)
  UNUSED(t)
  return 0.0;
}

double Crab::phiDerived2(const Bezier::Curve& curve, double t) const
{
  UNUSED(curve)
  UNUSED(t)
  return 0.0;
}
