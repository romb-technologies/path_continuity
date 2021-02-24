#ifndef WHEEL_H
#define WHEEL_H

#include<QTreeWidgetItem>
#include<QDoubleSpinBox>
#include<Bezier/declarations.h>
#include "MotionModes/motion_mode.h"

class Wheel : public QWidget, public QTreeWidgetItem
{
  Q_OBJECT
public:
  Wheel(int n, QTreeWidget* vehicle);

  Bezier::Point getR() const {return {sb_x->value()*100, sb_y->value()*100};}
  double getV() const {return sb_v->value()*100;}
  double getW() const {return sb_w->value() * M_PI / 180;}

  void expand();

  Bezier::Point Cw(const Bezier::Curve& curve, Romb::MotionModes::MotionMode* mm, double t);
  Bezier::Vector dCw(const Bezier::Curve& curve, Romb::MotionModes::MotionMode* mm, double t);
  Bezier::Vector d2Cw(const Bezier::Curve& curve, Romb::MotionModes::MotionMode* mm, double t);
  double delta(const Bezier::Curve& curve, Romb::MotionModes::MotionMode* mm, double t);
  double kappa(const Bezier::Curve& curve, Romb::MotionModes::MotionMode* mm, double t);
  double kappaDerivative(const Bezier::Curve& curve, Romb::MotionModes::MotionMode* mm, double t);
  double Rv(const Bezier::Curve& curve, Romb::MotionModes::MotionMode* mm, double t);
  double Rw(const Bezier::Curve& curve, Romb::MotionModes::MotionMode* mm, double t);
  double vMax(const Bezier::Curve& curve, Romb::MotionModes::MotionMode* mm, double t);

private:
  QTreeWidgetItem *r, *r_x, *r_y, *v_max, *w_max;
  QDoubleSpinBox *sb_x, *sb_y, *sb_v, *sb_w;

signals:
  void wheelChanged();
};

#endif // WHEEL_H
