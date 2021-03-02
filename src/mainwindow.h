#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "customscene.h"
#include <QMainWindow>

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

private:
  Ui::MainWindow* ui;
  CustomScene* scene;
  qCurve *curve_1, *curve_2, *curve_3;

  void applyContinuity();
  void updatePlot();

  struct Data{
    QVector<double> t, s1, s2, v1, v2, theta1, theta2;
    QVector<QVector<double>> delta1, delta2, v_w1, v_w2;
    QVector<double> v_sim1, v_sim2, w_sim1, w_sim2;
    QVector<QVector<double>> v_w_sim1, v_w_sim2, w_w_sim1, w_w_sim2;

    Data (int N, int samples, bool partial);
  };

  Data calculateData(const Bezier::Curve& curve_1, const Bezier::Curve& curve_2, bool partial = false);
};

#endif // MAINWINDOW_H
