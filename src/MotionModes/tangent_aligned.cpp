#include "MotionModes/tangent_aligned.h"

#include <Bezier/bezier.h>
#include <Bezier/polycurve.h>

using namespace Romb;
using namespace MotionModes;

TangentAligned::TangentAligned(double angle_offset) : angle_offset_(angle_offset) {type = "aligned";}

MotionMode* TangentAligned::clone() const { return new TangentAligned(angle_offset_); }

double TangentAligned::phi(const Bezier::Curve& curve, double t) const
{
  Bezier::Vector d1 = curve.derivativeAt(t);

  return wrapToPi(angle_offset_ + std::atan2(d1.y(), d1.x()));
}

double TangentAligned::phiDerived(const Bezier::Curve& curve, double t) const
{
  Bezier::Vector d1 = curve.derivativeAt(t);
  Bezier::Vector d2 = curve.derivativeAt(2, t);

  return (d1.x() * d2.y() - d2.x() * d1.y()) / d1.squaredNorm();
}

double TangentAligned::phiDerived2(const Bezier::Curve& curve, double t) const
{
  Bezier::Vector d1 = curve.derivativeAt(t);
  Bezier::Vector d2 = curve.derivativeAt(2, t);
  Bezier::Vector d3 = curve.derivativeAt(3, t);

  return (d3.y() * d1.x() - d3.x() * d1.y()) / d1.squaredNorm() + (d2.x() * d1.y() - d1.x() * d2.y()) *
                                                                      (2 * d1.x() * d2.x() + 2 * d1.y() * d2.y()) /
                                                                      std::pow(d1.squaredNorm(), 2);
}

Exponential::Exponential(double angle_offset, double n) : angle_offset_(angle_offset), n_(n) {type = n_ > 0 ? "exponential_1" : "exponential_2";}

MotionMode* Exponential::clone() const { return new Exponential(angle_offset_, n_); }

double Exponential::phi(const Bezier::Curve& curve, double t) const
{
  t = n_ > 0 ? std::pow(t, n_) : 1 - std::pow(1 - t, -n_);
  Bezier::Vector d1 = curve.derivativeAt(t);

  return wrapToPi(angle_offset_ + std::atan2(d1.y(), d1.x()));
}

double Exponential::phiDerived(const Bezier::Curve& curve, double t) const
{
  double t2 = n_ > 0 ? std::pow(t, n_) : 1 - std::pow(1 - t, -n_);
  Bezier::Vector d1 = curve.derivativeAt(t2);
  Bezier::Vector d2 = curve.derivativeAt(2, t2);

  return std::fabs(n_) * std::pow(n_ > 0 ? t : 1 - t, std::fabs(n_) - 1) * (d1.x() * d2.y() - d2.x() * d1.y()) / d1.squaredNorm();
}

double Exponential::phiDerived2(const Bezier::Curve& curve, double t) const
{
  Bezier::Vector d1 = curve.derivativeAt(t);
  Bezier::Vector d2 = curve.derivativeAt(2, t);
  Bezier::Vector d3 = curve.derivativeAt(3, t);

  return (d3.y() * d1.x() - d3.x() * d1.y()) / d1.squaredNorm() + (d2.x() * d1.y() - d1.x() * d2.y()) *
                                                                      (2 * d1.x() * d2.x() + 2 * d1.y() * d2.y()) /
                                                                      std::pow(d1.squaredNorm(), 2);
}
