#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDebug>
#include <QPrinter>

#include <cmath>
#include <iostream>

#include "MotionModes/crab.h"
#include "MotionModes/motion_mode.h"
#include "MotionModes/tangent_aligned.h"
#include "wheel.h"

#include <dlib/global_optimization.h>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow), scene(new CustomScene)
{
  ui->setupUi(this);

  ui->graphicsView->setScene(scene);
  new QGraphicsViewZoom(ui->graphicsView);

  Eigen::MatrixX2d cp1, cp2;
  cp1.resize(4, 2);
  cp2.resize(4, 2);

  cp1 << -1, 0, -1, -4. / 3, 1, -4. / 3, 1.5, -0.5;
  cp1 *= 300;
  cp2 << 1.5, -0.5, 1, 2, 2, 1, 3, 1;
  cp2 *= 150;
  cp2.col(0) *= 2;

  curve_1 = new qCurve(Bezier::Curve(cp1).splitCurve(0.5).second, ui, 1);
  curve_2 = new qCurve(cp2, ui, 2);

  curve_2->applyContinuity(*curve_1, {1.4});

  curve_1->elevateOrder();
  curve_1->elevateOrder();
  curve_1->elevateOrder();

  curve_2->elevateOrder();
  curve_2->elevateOrder();
  curve_2->elevateOrder();

  scene->addItem(curve_1);
  scene->addItem(curve_2);

  curve_1->setDraw_control_points(true);
  curve_2->setDraw_control_points(true);

  ui->graphicsView->centerOn(scene->itemsBoundingRect().center());

  // plots
  ui->plot2->yAxis2->setVisible(true);

  // add title layout element:
  //  ui->plot1->plotLayout()->insertRow(0);
  //  ui->plot2->plotLayout()->insertRow(0);
  //  ui->plot3->plotLayout()->insertRow(0);
  //  ui->plot1->setFont(QFont("Times", 12));
  //  ui->plot2->setFont(QFont("Times", 12));
  //  ui->plot3->setFont(QFont("Times", 12));
  //  ui->plot1->plotLayout()->addElement(0, 0, new QCPTextElement(ui->plot1, "Speed limit and speed profiles"));
  //  ui->plot2->plotLayout()->addElement(0, 0, new QCPTextElement(ui->plot2, "Angular velocities"));
  //  ui->plot3->plotLayout()->addElement(0, 0, new QCPTextElement(ui->plot3, "Vehicle orientation and steering
  //  angles"));

  // set labels:
  ui->plot1->xAxis->setLabelFont(QFont("Times", 10));
  ui->plot2->xAxis->setLabelFont(QFont("Times", 10));
  ui->plot1->xAxis->setLabel("Distance along path [m]");
  ui->plot2->xAxis->setLabel("Distance along path [m]");

  ui->plot1->yAxis->setLabelFont(QFont("Times", 10));
  ui->plot2->yAxis->setLabelFont(QFont("Times", 10));
  ui->plot2->yAxis2->setLabelFont(QFont("Times", 10));
  ui->plot1->yAxis->setLabel("Speed [m/s]");
  ui->plot2->yAxis->setLabel("Vehicle orientation and wheel steering angle [°]");
  ui->plot2->yAxis2->setLabel("Angular velocity [°/s]");

  // set ticks:
  ui->plot1->xAxis->setTickLabelFont(QFont("Times", 10));
  ui->plot2->xAxis->setTickLabelFont(QFont("Times", 10));
  ui->plot1->yAxis->setTickLabelFont(QFont("Times", 10));
  ui->plot2->yAxis->setTickLabelFont(QFont("Times", 10));
  ui->plot2->yAxis2->setTickLabelFont(QFont("Times", 10));

  // show legend
  ui->plot1->legend->setVisible(true);
  ui->plot2->legend->setVisible(true);
  ui->plot1->legend->setFont(QFont("Times", 10));
  ui->plot2->legend->setFont(QFont("Times", 10));
  ui->plot1->legend->setRowSpacing(-5);
  ui->plot2->legend->setRowSpacing(-5);
  ui->plot1->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom | Qt::AlignRight);
  ui->plot2->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop | Qt::AlignRight);

  // open in new window
  //  ui->plot1->setParent(nullptr);
  //  ui->plot2->setParent(nullptr);
  ui->plot1->setMinimumSize(400, 300);
  ui->plot2->setMinimumSize(400, 300);
  ui->plot1->show();
  ui->plot2->show();

  auto saveAs = []() {
    QFileDialog dialog(nullptr, "Export PDF");
    dialog.setNameFilter("PDF (*.pdf)");
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    dialog.setDefaultSuffix("pdf");
    if (dialog.exec())
      return dialog.selectedFiles().first();
    return QString();
  };

  connect(ui->plot1, &QCustomPlot::mouseRelease, this, [&](QMouseEvent* e) {
    if (e->button() == Qt::RightButton)
    {
      auto file_path = saveAs();
      if (file_path == QString())
        return;
      ui->plot1->savePdf(file_path, 0, 0, QCP::epNoCosmetic);
    }
  });
  connect(ui->plot2, &QCustomPlot::mouseRelease, this, [&](QMouseEvent* e) {
    if (e->button() == Qt::RightButton)
    {
      auto file_path = saveAs();
      if (file_path == QString())
        return;
      ui->plot2->savePdf(file_path, 0, 0, QCP::epNoCosmetic);
    }
  });

//    QPrinter printer(QPrinter::HighResolution);
//    printer.setPageSize(
//        QPageSize(scene->itemsBoundingRect().size().scaled(90, 90, Qt::AspectRatioMode::KeepAspectRatio),
//                  QPageSize::Unit::Point));
//    printer.setOrientation(QPrinter::Portrait);
//    printer.setOutputFormat(QPrinter::PdfFormat);
//    printer.setFullPage(true);
//    auto file_path = saveAs();
//    if (file_path == QString())
//      return;
//    printer.setOutputFileName(file_path);

//    QPainter p;

//    if (!p.begin(&printer))
//    {
//      qDebug() << "Error!";
//      return;
//    }

//    this->scene->render(&p);
//    p.end();

//    int sl = file_path.lastIndexOf('/') + 1;
//    QString file_path2 = file_path.left(sl) + "angle" + file_path.right(6);
//    ui->plot2->savePdf(file_path2, 0, 0, QCP::epNoCosmetic);
//    QString file_path3 = file_path.left(sl) + "speed" + file_path.right(6);
//    ui->plot1->savePdf(file_path3, 0, 0, QCP::epNoCosmetic);
//    // ovdje dodati export i ostalih

  ui->plot1->setInteraction(QCP::iRangeDrag, true);
  ui->plot2->setInteraction(QCP::iRangeDrag, true);

  ui->plot1->axisRect()->setRangeDrag(Qt::Horizontal);
  ui->plot2->axisRect()->setRangeDrag(Qt::Horizontal);

  ui->plot1->setInteraction(QCP::iRangeZoom, true);
  ui->plot2->setInteraction(QCP::iRangeZoom, true);

  ui->plot1->axisRect()->setRangeZoom(Qt::Horizontal);
  ui->plot2->axisRect()->setRangeZoom(Qt::Horizontal);


  // curves
  //  connect(ui->alpha1, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::applyContinuity);
  //  connect(ui->alpha2, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::applyContinuity);
  connect(ui->n1, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::applyContinuity);
  //  connect(ui->n2, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::applyContinuity);
  connect(ui->mm1, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::applyContinuity);
  //  connect(ui->mm2, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::applyContinuity);

  // vehicle tab
  ui->vehicle->header()->resizeSection(0, 150);
  ui->vehicle->header()->setSectionResizeMode(1, QHeaderView::Stretch);
  ui->vehicle->header()->setStretchLastSection(false);

  static auto addWheel = [&]() {
    Wheel* temp = new Wheel(ui->vehicle->topLevelItemCount(), ui->vehicle);
    ui->vehicle->addTopLevelItem(temp);
    temp->expand();
    connect(temp, &Wheel::wheelChanged, this, &MainWindow::updatePlot);
  };

  static auto removeWheel = [&]() { delete ui->vehicle->takeTopLevelItem(ui->vehicle->topLevelItemCount() - 1); };

  for (uint k = 0; k < ui->wheel_num->value(); k++)
    addWheel();
  connect(scene, &QGraphicsScene::changed, this, &MainWindow::updatePlot);

  connect(ui->wheel_num, qOverload<int>(&QSpinBox::valueChanged), this, [&](int n) {
    while (n > ui->vehicle->topLevelItemCount())
      addWheel();
    while (n < ui->vehicle->topLevelItemCount())
      removeWheel();
    updatePlot();
  });

  //  curve_1->setVisible(false);
  //  curve_2->setVisible(false);

  //  ui->wheel_num->setValue(6);

  //  static const QVector<QColor> matlab_colors(
  //      {{0, 114, 189}, {217, 83, 25}, {237, 177, 32}, {126, 47, 142}, {119, 172, 48}, {77, 190, 238}, {162, 20,
  //      47}});
  //  QPainterPath vehicle_frame;

  //  vehicle_frame = QPainterPath();
  //  vehicle_frame.moveTo(50, 0);
  //  vehicle_frame.lineTo(0, 0);
  //  vehicle_frame.lineTo(0, -50);
  //  vehicle_frame.lineTo(-3,-47);
  //  vehicle_frame.lineTo(0, -50);
  //  vehicle_frame.lineTo(3,-47);

  //  QVector<QString> w;
  //  w.push_back(u8"w\u2081");
  //  w.push_back(u8"w\u2082");
  //  w.push_back(u8"w\u2083");
  //  w.push_back(u8"w\u2084");
  //  w.push_back(u8"w\u2085");
  //  w.push_back(u8"w\u2086");

  //    scene->addPath(vehicle_frame, QPen(matlab_colors[0], 3));
  //    for(int k = 0; k < ui->wheel_num->value(); k++)
  //    {
  //      auto wheel = static_cast<Wheel*>(ui->vehicle->topLevelItem(k));
  //      auto R = wheel->getR();
  //      scene->addRect(QRect(R.x()*10-100, -(R.y()*10-50), 200, -100), QPen(matlab_colors[k+1], 1));
  //      scene->addLine(0,0,R.x()*10, -R.y()*10);
  //      auto x = scene->addText(w[k], QFont("Times", 24));
  //      x->setPos(R.x()*10, -R.y()*10);
  //    }

  //  scene->update();

  //  {
  //      QPrinter printer( QPrinter::HighResolution );
  //      printer.setPageSize(QPageSize(qCurve(cp2*5).boundingRect().size(), QPageSize::Unit::Point));
  //      printer.setOrientation( QPrinter::Portrait );
  //      printer.setOutputFormat( QPrinter::PdfFormat );
  //      printer.setFullPage(true);
  //      printer.setOutputFileName( "/home/mirko/Pictures/motion_modes/" + qCurve::getName());

  //      QPainter p;

  //      if( !p.begin( &printer ) )
  //      {
  //          qDebug() << "Error!";
  //          return;
  //      }
  //      this->scene->render( &p );
  //      p.end();
  //  }

  //  auto dC = curve_1->derivativeAt(1);
  //  ui->alpha2->setValue(std::atan2(dC.y(), dC.x()) * 180 / M_PI);
  //  ui->mm2->setCurrentIndex(2);
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::applyContinuity()
{
  Romb::MotionModes::MotionMode *mm_1, *mm_2;
  double alpha_1 = ui->alpha1->value() * M_PI / 180;
  double alpha_2 = ui->alpha2->value() * M_PI / 180;
  double n_1 = ui->n1->value();
  double n_2 = ui->n2->value();

  switch (ui->mm1->currentIndex())
  {
  case 0:
    mm_1 = new Romb::MotionModes::TangentAligned(alpha_1);
    break;
  case 1:
    mm_1 = new Romb::MotionModes::Crab(alpha_1);
    break;
  case 2:
    mm_1 = new Romb::MotionModes::Exponential(alpha_1, n_1);
    break;
  default:
    mm_1 = new Romb::MotionModes::Exponential(alpha_1, -n_1);
    break;
  }

  switch (ui->mm2->currentIndex())
  {
  case 0:
    mm_2 = new Romb::MotionModes::TangentAligned(alpha_2);
    break;
  case 1:
    mm_2 = new Romb::MotionModes::Crab(alpha_2);
    break;
  case 2:
    mm_2 = new Romb::MotionModes::Exponential(alpha_1, n_2);
    break;
  default:
    mm_2 = new Romb::MotionModes::Exponential(alpha_1, -n_2);
    break;
  }

  dlib::thread_pool tp(std::thread::hardware_concurrency());
  std::function<void(std::vector<double>, Bezier::Curve&, Bezier::Curve&)> applyParams;
  std::function<double(std::vector<double>)> evaluate = [&](std::vector<double> params) {
    Bezier::Curve temp1 = *curve_1;
    Bezier::Curve temp2 = *curve_2;
    applyParams(params, temp1, temp2);

    Data sim = calculateData(temp1, temp2, true);

    auto time = [](QVector<double> v, QVector<double> s) {
      double res{0.0};
      for (int k = 1; k < s.size(); k++)
        res += 2 * (s[k] - s[k - 1]) / (v[k] + v[k - 1]);
      return res;
    };

    return time(sim.v_sim1, sim.s1) + time(sim.v_sim2, sim.s2);
  };

  constexpr std::chrono::seconds opt_max_duration(15);

  if (mm_1->type == "aligned" && mm_2->type == "aligned")
  {
    ui->alpha2->setValue(alpha_1);
    applyParams = [](std::vector<double> params, Bezier::Curve& curve_1, Bezier::Curve& curve_2) {
      curve_2.applyContinuity(curve_1, {params[0], params[1], params[2]});
    };
    auto result = dlib::find_min_global(tp,
        [&](double x0, double x1, double x2) {
          return evaluate({x0, x1, x2});
        },
        {0.01, 0, 0}, {5, 5, 5}, opt_max_duration);

    applyParams(std::vector<double>(result.x.begin(), result.x.end()), *curve_1, *curve_2);
  }
  if ((mm_1->type == "aligned" || mm_1->type == "exponential_1") && mm_2->type == "crab")
  {
    auto dc = curve_1->derivativeAt(1);
    ui->alpha2->setValue((std::atan2(dc.y(), dc.x()) + alpha_1) * 180 / M_PI);
    applyParams = [](std::vector<double> params, Bezier::Curve& curve_1, Bezier::Curve& curve_2) {
      auto dc = curve_1.derivativeAt(1);
      curve_1.manipulateControlPoint(curve_1.order() - 1, curve_1.endPoints().second - params[0] * dc);
      curve_1.manipulateControlPoint(curve_1.order() - 2, curve_1.endPoints().second - params[1] * dc);
      curve_1.manipulateControlPoint(curve_1.order() - 3, curve_1.endPoints().second - params[2] * dc);

      curve_2.manipulateControlPoint(0, curve_1.endPoints().second);
      curve_2.manipulateControlPoint(1, curve_1.endPoints().second + params[3] * dc);
      curve_2.manipulateControlPoint(2, curve_1.endPoints().second + params[4] * dc);
    };

    auto result = dlib::find_min_global(tp,
        [&](double x0, double x1, double x2, double x3, double x4) {
          return evaluate({x0, x1, x2, x3, x4});
        },
        {0.1, 0, 0, 0.1, 0}, {5, 5, 5, 5, 5}, opt_max_duration);
    applyParams({result.x(0), result.x(1), result.x(2), result.x(3), result.x(4)}, *curve_1, *curve_2);
  }
  if ((mm_2->type == "aligned" || mm_2->type == "exponential_2") && mm_1->type == "crab")
  {
    auto dc = curve_1->derivativeAt(1);
    ui->alpha2->setValue((std::atan2(dc.y(), dc.x()) + alpha_1) * 180 / M_PI);
    applyParams = [](std::vector<double> params, Bezier::Curve& curve_2, Bezier::Curve& curve_1) {
      auto dc = curve_1.derivativeAt(0);
      curve_1.manipulateControlPoint(curve_1.order() - 1, curve_1.endPoints().second - params[0] * dc);
      curve_1.manipulateControlPoint(curve_1.order() - 2, curve_1.endPoints().second - params[1] * dc);
      curve_1.manipulateControlPoint(curve_1.order() - 3, curve_1.endPoints().second - params[2] * dc);

      curve_2.manipulateControlPoint(0, curve_1.endPoints().second);
      curve_2.manipulateControlPoint(1, curve_1.endPoints().second + params[3] * dc);
      curve_2.manipulateControlPoint(2, curve_1.endPoints().second + params[4] * dc);
    };

    auto result = dlib::find_min_global(tp,
        [&](double x0, double x1, double x2, double x3, double x4) {
          return evaluate({x0, x1, x2, x3, x4});
        },
        {0.1, 0, 0, 0.1, 0}, {5, 5, 5, 5, 5}, opt_max_duration);
    applyParams({result.x(0), result.x(1), result.x(2), result.x(3), result.x(4)}, *curve_1, *curve_2);
  }

  if (mm_1->type == "aligned" && mm_2->type == "exponential_2")
  {
    ui->alpha2->setValue(ui->alpha1->value());
    applyParams = [&](std::vector<double> params, Bezier::Curve& curve_1, Bezier::Curve& curve_2) {
      auto dc = curve_1.derivativeAt(1);
      curve_1.manipulateControlPoint(curve_1.order() - 1, curve_1.endPoints().second - params[0] * dc);
      curve_1.manipulateControlPoint(curve_1.order() - 2, curve_1.endPoints().second - params[1] * dc);

      curve_2.manipulateControlPoint(0, curve_1.endPoints().second);
      curve_2.manipulateControlPoint(1, curve_1.endPoints().second + params[2] * dc);
      curve_2.manipulateControlPoint(2, curve_1.endPoints().second + params[3] * dc);

      double beta_1 = curve_1.derivativeAt(1).norm() / curve_2.derivativeAt(0).norm();
      double beta_2 = curve_1.derivativeAt(1).norm() / curve_2.derivativeAt(0).norm();

      double dC3_scale = n_2 * n_2 * (beta_1 * beta_1 * beta_1);

      double beta_3 = (curve_2.derivativeAt(3, 0).norm() * dC3_scale -
                       beta_1 * beta_1 * beta_1 * curve_2.derivativeAt(2, 0).norm() -
                       2 * beta_1 * beta_2 * curve_2.derivativeAt(2, 0).norm()) /
                      curve_2.derivativeAt(0).norm();

      curve_2.applyContinuity(curve_1, {beta_1, beta_2, beta_3});
    };

    auto result = dlib::find_min_global(tp,
        [&](double x0, double x1, double x2, double x3) {
          return evaluate({x0, x1, x2, x3});
        },
        {0.0, 0, 0.0, 0.0}, {3, 3, 3, 3}, opt_max_duration);
    applyParams({result.x(0), result.x(1), result.x(2), result.x(3)}, *curve_1, *curve_2);
  }

  scene->update();
  updatePlot();
}

void MainWindow::updatePlot()
{
  const QString _max(u8"\u2098\u2090\u2093");
  const QString _sim(u8"\u209b\u1d62\u2098");

  Data data = calculateData(*curve_1, *curve_2);
  int N = ui->vehicle->topLevelItemCount();
  static const QVector<QColor> matlab_colors(
      {{0, 114, 189}, {217, 83, 25}, {237, 177, 32}, {126, 47, 142}, {119, 172, 48}, {77, 190, 238}, {162, 20, 47}});

  ////// PLOTANJE
  QPen pen;
  pen.setStyle(Qt::DashLine);
  static QCPItemStraightLine* infLine1 = new QCPItemStraightLine(ui->plot1);
  static QCPItemStraightLine* infLine2 = new QCPItemStraightLine(ui->plot2);
  infLine1->setPen(pen);
  infLine2->setPen(pen);
  infLine1->point1->setCoords(data.s1.back(), 0); // location of point 1 in plot coordinate
  infLine1->point2->setCoords(data.s1.back(), 1); // location of point 2 in plot coordinate
  infLine2->point1->setCoords(data.s1.back(), 0); // location of point 1 in plot coordinate
  infLine2->point2->setCoords(data.s1.back(), 1); // location of point 2 in plot coordinate

  /// plot brzina
  {
    constexpr int n_v = 4;
    while (ui->plot1->graphCount() < 4 * N + n_v)
      ui->plot1->addGraph();
    while (ui->plot1->graphCount() > 4 * N + n_v)
      ui->plot1->removeGraph(ui->plot1->graphCount() - 1);

    auto g_v1 = ui->plot1->graph(0);
    auto g_v2 = ui->plot1->graph(1);
    auto g_v_sim1 = ui->plot1->graph(2);
    auto g_v_sim2 = ui->plot1->graph(3);

    g_v1->setData(data.s1, data.v1, true);
    g_v2->setData(data.s2, data.v2, true);
    g_v_sim1->setData(data.s1, data.v_sim1, true);
    g_v_sim2->setData(data.s2, data.v_sim2, true);

    g_v1->setName("v" + _max);
    g_v2->removeFromLegend();
    g_v_sim1->setName("v" + _sim);
    g_v_sim2->removeFromLegend();

    pen.setStyle(Qt::SolidLine);
    pen.setColor(matlab_colors[0]);
    pen.setWidth(3);
    g_v1->setPen(pen);
    g_v2->setPen(pen);
    pen.setStyle(Qt::DashLine);
    g_v_sim1->setPen(pen);
    g_v_sim2->setPen(pen);

    for (int k = 0; k < N; k++)
    {
      auto g_w_v1 = ui->plot1->graph(n_v + 4 * k);
      auto g_w_v2 = ui->plot1->graph(n_v + 4 * k + 1);
      auto g_w_v_sim1 = ui->plot1->graph(n_v + 4 * k + 2);
      auto g_w_v_sim2 = ui->plot1->graph(n_v + 4 * k + 3);

      g_w_v1->setName("w" + QString::number(k + 1) + _max);
      g_w_v2->removeFromLegend();
      g_w_v_sim1->setName("w" + QString::number(k + 1) + _sim);
      g_w_v_sim2->removeFromLegend();

      g_w_v1->setData(data.s1, data.v_w1[k], true);
      g_w_v2->setData(data.s2, data.v_w2[k], true);
      g_w_v_sim1->setData(data.s1, data.v_w_sim1[k], true);
      g_w_v_sim2->setData(data.s2, data.v_w_sim2[k], true);

      pen.setWidth(1);
      pen.setColor(matlab_colors[k + 1]);
      pen.setStyle(Qt::SolidLine);
      g_w_v1->setPen(pen);
      g_w_v2->setPen(pen);
      pen.setStyle(Qt::DashLine);
      g_w_v_sim1->setPen(pen);
      g_w_v_sim2->setPen(pen);
    }

    ui->plot1->rescaleAxes();
    ui->plot1->yAxis->scaleRange(1.05);
    ui->plot1->replot();
  }

  /// plot kutnih brzina
  {
    constexpr int n_w = 4;
    while (ui->plot2->graphCount() < 4 * N + n_w)
    {
      ui->plot2->addGraph();
      ui->plot2->addGraph(ui->plot2->xAxis, ui->plot2->yAxis2);
    }
    while (ui->plot2->graphCount() > 4 * N + n_w)
      ui->plot2->removeGraph(ui->plot2->graphCount() - 1);

    auto g_theta1 = ui->plot2->graph(0);
    auto g_theta2 = ui->plot2->graph(2);
    auto g_w_sim1 = ui->plot2->graph(1);
    auto g_w_sim2 = ui->plot2->graph(3);

    g_theta1->setName(u8"\u03b8");
    g_theta2->removeFromLegend();
    g_w_sim1->setName(u8"\u03c9" + _sim);
    g_w_sim2->removeFromLegend();

    g_theta1->setData(data.s1, data.theta1, true);
    g_theta2->setData(data.s2, data.theta2, true);
    g_w_sim1->setData(data.s1, data.w_sim1, true);
    g_w_sim2->setData(data.s2, data.w_sim2, true);

    pen.setStyle(Qt::SolidLine);
    pen.setColor(matlab_colors[0]);
    pen.setWidth(3);
    g_theta1->setPen(pen);
    g_theta2->setPen(pen);
    pen.setStyle(Qt::DashLine);
    g_w_sim1->setPen(pen);
    g_w_sim2->setPen(pen);

    for (int k = 0; k < N; k++)
    {
      auto g_delta1 = ui->plot2->graph(n_w + 4 * k + 0);
      auto g_delta2 = ui->plot2->graph(n_w + 4 * k + 2);
      auto g_w_w_sim1 = ui->plot2->graph(n_w + 4 * k + 1);
      auto g_w_w_sim2 = ui->plot2->graph(n_w + 4 * k + 3);

      g_delta1->setName(u8"\u03b4" + QString::number(k + 1));
      g_delta2->removeFromLegend();
      g_w_w_sim1->setName(u8"\u03c9w" + QString::number(k + 1) + _sim);
      g_w_w_sim2->removeFromLegend();

      g_delta1->setData(data.s1, data.delta1[k], true);
      g_delta2->setData(data.s2, data.delta2[k], true);
      g_w_w_sim1->setData(data.s1, data.w_w_sim1[k], true);
      g_w_w_sim2->setData(data.s2, data.w_w_sim2[k], true);

      pen.setStyle(Qt::SolidLine);
      pen.setWidth(1);
      pen.setColor(matlab_colors[k + 1]);
      g_delta1->setPen(pen);
      g_delta2->setPen(pen);
      pen.setStyle(Qt::DashLine);
      g_w_w_sim1->setPen(pen);
      g_w_w_sim2->setPen(pen);
    }

    ui->plot2->rescaleAxes();
    ui->plot2->yAxis->scaleRange(1.05);
    ui->plot2->yAxis2->scaleRange(1.05);
    ui->plot2->replot();
  }

  auto print = [](double x) {
    std::ostringstream res;
    res.precision(3);

    if (x < 0)
      res << "{" << std::fixed << x << "}";
    else
      res << "  " << std::fixed << x << " ";
    return res;
  };

  std::stringstream bezier1, bezier2;
  auto cp1 = curve_1->controlPoints();
  auto cp2 = curve_1->controlPoints();
  for (auto& p : cp1)
  {
    p = {std::round(p.x() * 10) / 1000., std::round(p.y() * 10) / 1000.};
    bezier1 << "( " << print(p.x()).str() << ",&\\ " << print(p.y()).str() << " ) \\\\\n";
  }
  for (auto& p : cp1)
  {
    p = {std::round(p.x() * 10) / 1000., std::round(p.y() * 10) / 1000.};
    bezier2 << "( " << print(p.x()).str() << ",&\\ " << print(p.y()).str() << " ) \\\\\n";
  }
  cp1.back() = {std::round(cp1.back().x() * 10) / 1000., std::round(cp1.back().y() * 10) / 1000.};
  bezier1 << "( " << print(cp1.back().x()).str() << ",&\\ " << print(cp1.back().y()).str() << " )";
  cp2.back() = {std::round(cp2.back().x() * 10) / 1000., std::round(cp2.back().y() * 10) / 1000.};
  bezier2 << "( " << print(cp2.back().x()).str() << ",&\\ " << print(cp2.back().y()).str() << " )";

  ui->bezier1->setText(QString::fromStdString(bezier1.str()));
  ui->bezier2->setText(QString::fromStdString(bezier2.str()));
}

MainWindow::Data MainWindow::calculateData(const Bezier::Curve& curve_1, const Bezier::Curve& curve_2, bool partial)
{
  ///////// DATA
  ///
  ///
  ///
  ///
  constexpr double step = 0.005;
  constexpr int samples = static_cast<int>(1. / step + 1);
  constexpr double a = 0.5;
  int N = ui->vehicle->topLevelItemCount();
  Romb::MotionModes::MotionMode *mm_1, *mm_2;
  double alpha_1 = ui->alpha1->value() * M_PI / 180;
  double alpha_2 = ui->alpha2->value() * M_PI / 180;
  double n_1 = ui->n1->value();
  double n_2 = ui->n2->value();
  double v_max_1 = ui->vmax1->value() * 100;
  double v_max_2 = ui->vmax2->value() * 100;
  switch (ui->mm1->currentIndex())
  {
  case 0:
    mm_1 = new Romb::MotionModes::TangentAligned(alpha_1);
    break;
  case 1:
    mm_1 = new Romb::MotionModes::Crab(alpha_1);
    break;
  case 2:
    mm_1 = new Romb::MotionModes::Exponential(alpha_1, n_1);
    break;
  default:
    mm_1 = new Romb::MotionModes::Exponential(alpha_1, -n_1);
    break;
  }
  switch (ui->mm2->currentIndex())
  {
  case 0:
    mm_2 = new Romb::MotionModes::TangentAligned(alpha_2);
    break;
  case 1:
    mm_2 = new Romb::MotionModes::Crab(alpha_2);
    break;
  case 2:
    mm_2 = new Romb::MotionModes::Exponential(alpha_1, n_2);
    break;
  default:
    mm_2 = new Romb::MotionModes::Exponential(alpha_1, -n_2);
    break;
  }

  Data results(N, samples, partial);

  auto& t = results.t;
  auto& s1 = results.s1;
  auto& s2 = results.s2;
  auto& v1 = results.v1;
  auto& v2 = results.v2;
  auto& v_sim1 = results.v_sim1;
  auto& v_sim2 = results.v_sim2;
  auto& w_sim1 = results.w_sim1;
  auto& w_sim2 = results.w_sim2;
  auto& theta1 = results.theta1;
  auto& theta2 = results.theta2;
  auto& delta1 = results.delta1;
  auto& delta2 = results.delta2;
  auto& v_w1 = results.v_w1;
  auto& v_w2 = results.v_w2;
  auto& v_w_sim1 = results.v_w_sim1;
  auto& v_w_sim2 = results.v_w_sim2;
  auto& w_w_sim1 = results.w_w_sim1;
  auto& w_w_sim2 = results.w_w_sim2;

  double S = curve_1.length() / 100;

  /////////// izracun limita brzine
  ///
  ///
  ///
  ///
  for (int idx = 0; idx < samples; idx++)
  {
    t[idx] = idx * step;
    s1[idx] = curve_1.length(t[idx]) / 100;
    s2[idx] = S + curve_2.length(t[idx]) / 100;
    v1[idx] = v_max_1;
    v2[idx] = v_max_2;

    for (int k = 0; k < N; k++)
    {
      auto wheel = static_cast<Wheel*>(ui->vehicle->topLevelItem(k));

      // single wheel limit #1
      v_w1[k][idx] = wheel->vMax(curve_1, mm_1, t[idx]);
      if (v_w1[k][idx] < v1[idx])
        v1[idx] = v_w1[k][idx];

      // single wheel limit #2
      v_w2[k][idx] = wheel->vMax(curve_2, mm_2, t[idx]);
      if (v_w2[k][idx] < v2[idx])
        v2[idx] = v_w2[k][idx];
    }
    v1[idx] /= 100;
    v2[idx] /= 100;

    // combined speed limit
    for (int k = 0; k < N; k++)
    {
      v_w1[k][idx] = static_cast<Wheel*>(ui->vehicle->topLevelItem(k))->Rv(curve_1, mm_1, t[idx]) * v1[idx];
      v_w2[k][idx] = static_cast<Wheel*>(ui->vehicle->topLevelItem(k))->Rv(curve_2, mm_2, t[idx]) * v2[idx];
    }
  }

  //////// simulacija brzine
  ///
  ///
  ///
  ///
  QVector<double> v_full = v1;
  QVector<double> s_full = s1;

  v_full.append(v2);
  s_full.append(s2);

  int last_idx = std::numeric_limits<int>::infinity();

  auto fill_sim = [&](double v, int idx) {
    auto fill_all = [&](double v, int idx) {
      auto& v_w_sim = idx < samples ? v_w_sim1 : v_w_sim2;
      auto& curve = idx < samples ? curve_1 : curve_2;
      auto& mm = idx < samples ? mm_1 : mm_2;
      idx = idx < samples ? idx : idx - samples;

      for (int k = 0; k < N; k++)
      {
        auto wheel = static_cast<Wheel*>(ui->vehicle->topLevelItem(k));
        v_w_sim[k][idx] = wheel->Rv(curve, mm, t[idx]) * v;
      }
    };

    for (int k = last_idx + 1; k <= idx; k++)
    {
      if (k == samples)
      {
        v_sim2[0] = v_sim1[k - 1];
        fill_all(v_sim2[0], k);
      }
      else
      {
        double dS = s_full[k] - s_full[k - 1];
        auto& v_sim = k < samples ? v_sim1 : v_sim2;
        int temp_idx = k < samples ? k : k - samples;
        if (v > v_sim[temp_idx - 1])
          v_sim[temp_idx] = std::min(v, std::sqrt(v_sim[temp_idx - 1] * v_sim[temp_idx - 1] + 2 * a * dS));
        else
          v_sim[temp_idx] = std::max(v, std::sqrt(v_sim[temp_idx - 1] * v_sim[temp_idx - 1] - 2 * a * dS));
        fill_all(v_sim[temp_idx], k);
      }
    }

    last_idx = idx;
  };

  std::function<void(int, double, int, double)> d_and_c = [&](int idx1, double v_1, int idx3, double v_3) {
    if (idx1 + 1 == idx3)
    {
      fill_sim(v_1, idx1);
      return;
    }

    int idx2 =
        static_cast<int>(std::min_element(v_full.begin() + idx1 + 1, v_full.begin() + idx3 - 1) - v_full.begin());

    double v_reachable_1 = std::sqrt(v_1 * v_1 + 2 * a * (s_full[idx2] - s_full[idx1]));
    double v_reachable_2 = std::sqrt(v_3 * v_3 + 2 * a * (s_full[idx3] - s_full[idx2]));

    if (v_reachable_1 > std::min(v_reachable_2, v_full[idx2]))
      d_and_c(idx1, v_1, idx2, std::min({v_reachable_1, v_reachable_2, v_full[idx2]}));
    else
      fill_sim(v_1, idx1);

    if (v_reachable_2 > std::min(v_reachable_1, v_full[idx2]))
      d_and_c(idx2, std::min({v_reachable_1, v_reachable_2, v_full[idx2]}), idx3, v_3);
    else
      fill_sim(std::min({v_reachable_1, v_reachable_2, v_full[idx2]}), idx2);
  };

  d_and_c(0, 0.0, 2 * samples - 1, 0.0);
  fill_sim(0.0, 2 * samples - 1);

  // return if only speed sim is wanted
  if (partial)
    return results;

  /////////// angular velocity and orientation sim
  ///
  ///
  ///
  ///
  ///

  for (int idx = 0; idx < samples; idx++)
  {
    theta1[idx] = mm_1->phi(curve_1, t[idx]) * 180 / M_PI;
    theta2[idx] = mm_2->phi(curve_2, t[idx]) * 180 / M_PI;
    w_sim1[idx] =
        v_sim1[idx] * mm_1->phiDerived(curve_1, t[idx]) / curve_1.derivativeAt(t[idx]).norm() * 100 * 180 / M_PI;
    w_sim2[idx] =
        v_sim2[idx] * mm_2->phiDerived(curve_2, t[idx]) / curve_2.derivativeAt(t[idx]).norm() * 100 * 180 / M_PI;

    for (int k = 0; k < N; k++)
    {
      auto wheel = static_cast<Wheel*>(ui->vehicle->topLevelItem(k));
      delta1[k][idx] = wheel->delta(curve_1, mm_1, t[idx]) * 180 / M_PI;
      delta2[k][idx] = wheel->delta(curve_2, mm_2, t[idx]) * 180 / M_PI;
      w_w_sim1[k][idx] = v_sim1[idx] * wheel->Rw(curve_1, mm_1, t[idx]) * 100 * 180 / M_PI;
      w_w_sim2[k][idx] = v_sim2[idx] * wheel->Rw(curve_2, mm_2, t[idx]) * 100 * 180 / M_PI;
    }
  }

  return results;
}

MainWindow::Data::Data(int N, int samples, bool partial)
{
  t.resize(samples);
  s1.resize(samples);
  s2.resize(samples);
  v1.resize(samples);
  v2.resize(samples);

  v_sim1.resize(samples);
  v_sim2.resize(samples);
  v_w1.resize(N);
  v_w2.resize(N);
  v_w_sim1.resize(N);
  v_w_sim2.resize(N);
  if (!partial)
  {
    theta1.resize(samples);
    theta2.resize(samples);
    w_sim1.resize(samples);
    w_sim2.resize(samples);
    delta1.resize(N);
    delta2.resize(N);
    w_w_sim1.resize(N);
    w_w_sim2.resize(N);
  }

  for (int k = 0; k < N; k++)
  {
    v_w1[k].resize(samples);
    v_w2[k].resize(samples);
    v_w_sim1[k].resize(samples);
    v_w_sim2[k].resize(samples);
    if (!partial)
    {
      delta1[k].resize(samples);
      delta2[k].resize(samples);
      w_w_sim1[k].resize(samples);
      w_w_sim2[k].resize(samples);
    }
  }
}
