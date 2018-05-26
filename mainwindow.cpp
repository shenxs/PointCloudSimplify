#include "mainwindow.h"
#include "iostream"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QLabel>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
  QSurfaceFormat format;
  format.setDepthBufferSize(24);
  format.setStencilBufferSize(8);
  format.setVersion(3, 3);
  format.setProfile(QSurfaceFormat::CoreProfile);
  glwidget = this->findChild<MyGLWidget *>("glwidget");
  glwidget->setFormat(format);

  xSlider = createVerticalSlider();
  ySlider = createVerticalSlider();
  zSlider = createVerticalSlider();

  ui->horizontalLayout->addWidget(xSlider);
  ui->horizontalLayout->addWidget(ySlider);
  ui->horizontalLayout->addWidget(zSlider);

  connect(xSlider, &QSlider::valueChanged, glwidget, &MyGLWidget::setXRotation);
  connect(glwidget, &MyGLWidget::xRotationChanged, xSlider, &QSlider::setValue);
  connect(ySlider, &QSlider::valueChanged, glwidget, &MyGLWidget::setYRotation);
  connect(glwidget, &MyGLWidget::yRotationChanged, ySlider, &QSlider::setValue);
  connect(zSlider, &QSlider::valueChanged, glwidget, &MyGLWidget::setZRotation);
  connect(glwidget, &MyGLWidget::zRotationChanged, zSlider, &QSlider::setValue);

  ui->cellLength->setVisible(false);
  ui->curve->setVisible(false);
  statusLabel = new QLabel();
  ui->statusBar->addWidget(statusLabel);

  connect(glwidget, &MyGLWidget::celllengthChange, this, &MainWindow::setCell);
  connect(glwidget, &MyGLWidget::curvChange, this, &MainWindow::setCurv);

  connect(glwidget, &MyGLWidget::showCurrentStatus, this,
          &MainWindow::setStatusMessage);
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::on_openfile_triggered() {
  QString modelFilePath = QFileDialog::getOpenFileName(
      this, tr("选择模型文件"), "./", tr("点云模型 (*.xyz)"));
  std::cout << modelFilePath.toStdString() << std::endl;
  glwidget->setPointCloud(modelFilePath.toStdString());
  ui->cellLength->setVisible(false);
  ui->curve->setVisible(false);
}

void MainWindow::on_curvSimplify_triggered() {
  currentSimplify = curva;
  ui->cellLength->setVisible(true);
  ui->curve->setVisible(true);
  glwidget->curvSimplify();
}

QSlider *MainWindow::createVerticalSlider() {
  QSlider *slider = new QSlider(Qt::Vertical);
  slider->setRange(0, 360 * 16);
  slider->setSingleStep(16);
  slider->setPageStep(15 * 16);
  slider->setTickInterval(15 * 16);
  slider->setTickPosition(QSlider::TicksRight);
  return slider;
}

void MainWindow::on_curve_valueChanged(double arg1) {
  double cell = ui->cellLength->value();
  glwidget->curvSimplify(cell, arg1);
}

void MainWindow::setCell(double val) {
  ui->cellLength->setVisible(true);
  ui->cellLength->setValue(val);
}

void MainWindow::setCurv(float val) {
  ui->curve->setVisible(true);
  ui->curve->setValue(val);
}

void MainWindow::on_cellLength_valueChanged(double arg1) {

  switch (currentSimplify) {
  case curva:
    glwidget->curvSimplify(arg1, ui->curve->value());
    break;
  case average:
    glwidget->averageSimplify(arg1);
    break;
  case averagedbscan:
    glwidget->averageSimplifyDBSCAN(arg1);
    break;
  }
}

void MainWindow::setStatusMessage(const QString &mess, int timeout) {
  statusLabel->setText(mess);
}

void MainWindow::on_action_triggered() {
  currentSimplify = average;
  ui->cellLength->setVisible(true);
  ui->curve->setVisible(false);
  glwidget->averageSimplify();
}

void MainWindow::on_action_2_triggered() {
  currentSimplify = averagedbscan;
  glwidget->averageSimplifyDBSCAN();
}
