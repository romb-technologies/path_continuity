#include "wheel.h"

#include "Bezier/bezier.h"

inline double wrapToPi(double angle)
{
  while(angle > M_PI)
    angle -= 2*M_PI;
  while(angle <= -M_PI)
    angle += 2*M_PI;
  return angle;
}

Wheel::Wheel(int n, QTreeWidget* vehicle)
{
  this->setText(0, QString("Wheel #") + QString::number(n + 1));
  r = new QTreeWidgetItem(this);
  r_x = new QTreeWidgetItem(r);
  r_y = new QTreeWidgetItem(r);
  v_max = new QTreeWidgetItem(this);
  w_max = new QTreeWidgetItem(this);

  r->setText(0, "Position (r)");
  r_x->setText(0, "X");
  r_y->setText(0, "Y");
  v_max->setText(0, "v_max");
  w_max->setText(0, "w_max");

  r_x->setText(2, "[m]");
  r_y->setText(2, "[m]");
  v_max->setText(2, "[m/s]");
  w_max->setText(2, "[°/s]");

  vehicle->setItemWidget(r_x, 1, sb_x = new QDoubleSpinBox);
  vehicle->setItemWidget(r_y, 1, sb_y = new QDoubleSpinBox);
  vehicle->setItemWidget(v_max, 1, sb_v = new QDoubleSpinBox);
  vehicle->setItemWidget(w_max, 1, sb_w = new QDoubleSpinBox);

  sb_x->setSingleStep(0.1);
  sb_y->setSingleStep(0.1);
  sb_v->setSingleStep(0.1);
  sb_w->setSingleStep(0.1);

  sb_x->setMinimum(-10);
  sb_x->setMaximum(10);
  sb_y->setMinimum(-10);
  sb_y->setMaximum(10);
  sb_v->setMinimum(0);
  sb_w->setMinimum(-180);
  sb_w->setMaximum(180);

  double a, b;
  double alpha = 0.4, beta = std::sqrt(3)*alpha;
  switch (n)
  {
  case 0:
    a = beta;
    b = -alpha;
    break;
  case 1:
    a = -beta;
    b = alpha;
    break;
  case 2:
    a = beta;
    b = alpha;
    break;
  case 3:
    a = -beta;
    b = -alpha;
    break;
  case 4:
    a = 0;
    b = -2*alpha;
    break;
  case 5:
    a = 0.0;
    b = 2*alpha;
    break;
  }
  sb_x->setValue(a + alpha/2);
  sb_y->setValue(b);
  sb_v->setValue(1.7);
  sb_w->setValue(45);

  connect(sb_x, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { emit Wheel::wheelChanged(); });
  connect(sb_y, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { emit Wheel::wheelChanged(); });
  connect(sb_v, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { emit Wheel::wheelChanged(); });
  connect(sb_w, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { emit Wheel::wheelChanged(); });
}

void Wheel::expand()
{
  this->setExpanded(true);
  r->setExpanded(true);
}

Bezier::Point Wheel::Cw(const Bezier::Curve& curve, Romb::MotionModes::MotionMode* mm, double t)
{
  return curve.valueAt(t) + mm->Rot(curve, t) * getR();
}
Bezier::Vector Wheel::dCw(const Bezier::Curve& curve, Romb::MotionModes::MotionMode* mm, double t)
{
  return curve.derivativeAt(t) + mm->RotDerived(curve, t) * getR();
}

Bezier::Vector Wheel::d2Cw(const Bezier::Curve& curve, Romb::MotionModes::MotionMode* mm, double t)
{
  return curve.derivativeAt(2, t) + mm->RotDerived2(curve, t) * getR();
}

double Wheel::delta(const Bezier::Curve& curve, Romb::MotionModes::MotionMode* mm, double t)
{
  auto d = dCw(curve, mm, t);
  return wrapToPi(std::atan2(d.y(), d.x()) - mm->phi(curve, t));
}

double Wheel::kappa(const Bezier::Curve &curve, Romb::MotionModes::MotionMode *mm, double t)
{
  auto d1 = dCw(curve, mm, t);
  auto d2 = d2Cw(curve, mm, t);
  return (d1.x() * d2.y() - d1.y() * d2.x()) / std::pow(d1.norm(), 3);
}

double Wheel::Rv(const Bezier::Curve& curve, Romb::MotionModes::MotionMode* mm, double t)
{
  return dCw(curve, mm, t).norm() / curve.derivativeAt(t).norm();
}

double Wheel::Rw(const Bezier::Curve& curve, Romb::MotionModes::MotionMode* mm, double t)
{
  auto d1 = dCw(curve, mm, t);
  return ((kappa(curve, mm, t) * d1.norm() - mm->phiDerived(curve, t))) / curve.derivativeAt(t).norm();
}

double Wheel::vMax(const Bezier::Curve& curve, Romb::MotionModes::MotionMode* mm, double t)
{
  return std::min(getV() / Rv(curve, mm, t), std::fabs(getW() / Rw(curve, mm, t)));
}
