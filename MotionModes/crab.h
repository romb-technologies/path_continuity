#ifndef CRAB
#define CRAB

#include "MotionModes/motion_mode.h"

namespace Romb
{
namespace MotionModes
{

class Crab : public MotionMode
{
private:
  double phi_;

public:
  Crab(double phi);

  MotionMode* clone() const override;

  double phi(const Bezier::Curve& curve, double t) const override;
  double phiDerived(const Bezier::Curve& curve, double t) const override;
  double phiDerived2(const Bezier::Curve& curve, double t) const override;
};

} // namespace MotionModes
} // namespace Romb
#endif // CRAB
