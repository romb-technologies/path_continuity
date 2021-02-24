#include "qcurve.h"

#include "MotionModes/crab.h"
#include "MotionModes/tangent_aligned.h"

#include <QDebug>
#include <QPainter>
#include <QPen>

#include "wheel.h"

#include "MotionModes/crab.h"
#include "MotionModes/motion_mode.h"
#include "MotionModes/tangent_aligned.h"

//static QRectF bb;
static double s = 8;
static double steps = 15;
////  auto mm = Romb::MotionModes::TangentAligned(0./180*M_PI); static QString name = "ta_1.pdf";
//// auto mm = Romb::MotionModes::TangentAligned(std::atan2(20,10)); static QString name = "diff.pdf";
//// static auto mm = Romb::MotionModes::TangentAligned(32./180*M_PI); static QString name = "ta_2.pdf";
////  auto mm = Romb::MotionModes::Crab(0./180*M_PI); static QString name = "cr_1.pdf";
////  auto mm = Romb::MotionModes::Crab(-32./180*M_PI); static QString name = "cr_2.pdf";
// auto mm = Romb::MotionModes::Exponential(0, -1.7);
// static QString name = "exp_1.pdf";
////  auto mm = Romb::MotionModes::Exponential(0, 3); static QString name = "exp_1.pdf";

static QPainterPath arrow, v, w;

static Bezier::Point w1, w2;

void qCurve::setDraw_control_points(bool value) { draw_control_points = value; }

void qCurve::setDraw_curvature_radious(bool value) { draw_curvature_radious = value; }

bool qCurve::getDraw_control_points() const { return draw_control_points; }

bool qCurve::getDraw_curvature_radious() const { return draw_curvature_radious; }

bool qCurve::getLocked() const { return locked; }

void qCurve::setLocked(bool value) { locked = value; }

// QString qCurve::getName() { return name; }

qCurve::qCurve(const Eigen::MatrixX2d& points, Ui::MainWindow* main_window, int n)
    : QGraphicsItem(), Bezier::Curve(points), ui(main_window), mm_n(n)
{
  //  w1 << 15 * s, 5 * s;
  //  w2 << -5 * s, -5 * s;

  //  v.addRect(-10 * s, -10 * s, 30 * s, 20 * s);
  //  w.addRoundedRect(-3 * s, -1.5 * s, 6 * s, 3 * s, 1.5 * s, 1.5 * s);
  //  arrow.moveTo(0, 0);
  //  arrow.lineTo(5 * s, 0);
  //  arrow.lineTo(4 * s, 1 * s);
  //  arrow.moveTo(5 * s, 0);
  //  arrow.lineTo(4 * s, -1 * s);

  //  for (double t = 0; t < 1; t = iterateByLength(t, length() / steps))
  //  {
  //    QPoint point(valueAt(t).x(), valueAt(t).y());
  //    QTransform tr;
  //    tr.rotateRadians(mm_n.phi(*this, t));
  //    bb = bb.united(tr.map(v).translated(point).boundingRect());
  //  }
}

int qCurve::type() const { return QGraphicsItem::UserType + 1; }

void qCurve::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
  Q_UNUSED(option)
  Q_UNUSED(widget)

  int N = ui->vehicle->topLevelItemCount();
  Romb::MotionModes::MotionMode* mm;
  double alpha, n;
  switch (mm_n)
  {
  case 1:
    alpha = ui->alpha1->value() * M_PI / 180;
    n = ui->n1->value();
    break;
  default:
  case 2:
    alpha = ui->alpha2->value() * M_PI / 180;
    n = ui->n2->value();
  }

  switch (mm_n == 1 ? ui->mm1->currentIndex() : ui->mm2->currentIndex())
  {
  case 0:
    mm = new Romb::MotionModes::TangentAligned(alpha);
    break;
  case 1:
    mm = new Romb::MotionModes::Crab(alpha);
    break;
  case 2:
    mm = new Romb::MotionModes::Exponential(alpha, n);
    break;
  default:
    mm = new Romb::MotionModes::Exponential(alpha, -n);
    break;
  }

  //  auto drawVehicle = [&](double t) {
  //    Bezier::Point dcv = derivativeAt(t);
  //    Bezier::Point dcw1 = dcv + mm.RotDerived(*this, t) * w1;
  //    Bezier::Point dcw2 = dcv + mm.RotDerived(*this, t) * w2;
  //    QPoint point(valueAt(t).x(), valueAt(t).y());

  //    QTransform tr, trw1, trw2;
  //    ;

  //    tr.rotateRadians(mm.phi(*this, t));
  //    trw1.rotateRadians(std::atan2(dcw1.y(), dcw1.x()));
  //    trw2.rotateRadians(std::atan2(dcw2.y(), dcw2.x()));

  //    painter->drawPath(tr.map(v).translated(point));
  //    painter->setBrush(Qt::gray);
  //    painter->drawPath(trw1.map(w).translated(Cw(t, w1).x(), Cw(t, w1).y()));
  //    painter->drawPath(trw2.map(w).translated(Cw(t, w2).x(), Cw(t, w2).y()));
  //    painter->setBrush(Qt::transparent);

  //    QPen pen = painter->pen();
  //    QPen pen2 = pen;
  //    pen2.setWidth(s / 2);
  //    pen2.setColor(Qt::red);
  //    painter->setPen(pen2);
  //    painter->drawPath(tr.map(arrow).translated(point));
  //    pen2.setColor(Qt::blue);
  //    painter->setPen(pen2);
  //    painter->drawPath(trw1.map(arrow).translated(Cw(t, w1).x(), Cw(t, w1).y()));
  //    painter->drawPath(trw2.map(arrow).translated(Cw(t, w2).x(), Cw(t, w2).y()));
  //    painter->setPen(pen);
  //  };

  static const QVector<QColor> matlab_colors(
      {{0, 114, 189}, {217, 83, 25}, {237, 177, 32}, {126, 47, 142}, {119, 172, 48}, {77, 190, 238}, {162, 20, 47}});

  setFlag(GraphicsItemFlag::ItemIsSelectable, true);

  painter->setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform, true);

  QPen pen;
  pen.setColor(matlab_colors[0]);
  pen.setStyle(isSelected() ? Qt::DashDotLine : Qt::SolidLine);
//  pen.setColor(getLocked() ? Qt::red : Qt::black);
  pen.setWidth(10);
  painter->setPen(pen);
  QPainterPath curve;
  auto poly = polyline(1.0001, 0.001);
  curve.moveTo(poly[0].x(), -poly[0].y());
  for (uint k = 1; k < poly.size(); k++)
    curve.lineTo(poly[k].x(), -poly[k].y());
  painter->drawPath(curve);
  bb = curve.boundingRect();

  pen.setWidth(1);
  for (int k = 0; k < N; k++)
  {
    pen.setColor(matlab_colors[k + 1]);
    painter->setPen(pen);
    QPainterPath wheel_path;
    auto wheel = static_cast<Wheel*>(ui->vehicle->topLevelItem(k));
    auto cw = wheel->Cw(*this, mm, 0.0);
    wheel_path.moveTo(cw.x(), -cw.y());
    for (double t = 0; t < 1; t += 0.005)
    {
      cw = wheel->Cw(*this, mm, t);
      wheel_path.lineTo(cw.x(), -cw.y());
    }
    cw = wheel->Cw(*this, mm, 1.0);
    wheel_path.lineTo(cw.x(), -cw.y());
    painter->drawPath(wheel_path);
    bb = bb.united(wheel_path.boundingRect());
  }

  //  painter->setPen(Qt::black);
  //  double len = length();
  //  for(double t = 0; t < 1; t+=1./steps)//t=iterateByLength(t, len/steps))
  //     drawVehicle(t);
  ////  drawVehicle(1.0);

  if (draw_control_points)
  {
    const int d = 12;
    painter->setBrush(QBrush(Qt::blue, Qt::SolidPattern));
    Bezier::PointVector points = controlPoints();
    for (uint k = 1; k < points.size(); k++)
    {
      painter->setPen(Qt::blue);
      painter->drawEllipse(QRectF(points[k - 1].x() - d / 2, -points[k - 1].y() - d / 2, d, d));
      painter->setPen(QPen(QBrush(Qt::gray), 1, Qt::DotLine));
      painter->drawLine(QLineF(points[k - 1].x(), -points[k - 1].y(), points[k].x(), -points[k].y()));
      bb = bb.united(QRectF(points[k - 1].x() - d / 2, -points[k - 1].y() - d / 2, d, d));
    }
    painter->setPen(Qt::red);
    painter->setBrush(QBrush(Qt::red, Qt::SolidPattern));
    painter->drawEllipse(QRectF(points.back().x() - d / 2, -points.back().y() - d / 2, d, d));
    painter->drawEllipse(QRectF(points.front().x() - d / 2, -points.front().y() - d / 2, d, d));
    bb = bb.united(QRectF(points.back().x() - d / 2, -points.back().y() - d / 2, d, d));
  }

  if (draw_curvature_radious)
  {
    for (Bezier::Parameter t = 1.0 / 100; t <= 1.0; t += 1.0 / 200)
    {
      painter->setPen(QColor(abs(255 * (0.5 - t)), (int)(255 * t), (int)(255 * (1 - t))));
      auto p = valueAt(t);
      auto tangent = tangentAt(t);
      Bezier::Point normal(-tangent.y(), tangent.x());
      double kappa = curvatureAt(t);
      auto n1 = p + normal * kappa * 100;
      auto n2 = p - normal * kappa * 100;
      painter->drawLine(QLineF(n1.x(), n1.y(), n2.x(), n2.y()));
    }
  }
}

QRectF qCurve::boundingRect() const
{
  return bb.adjusted(8, -20, 8, 8);
}
