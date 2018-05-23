#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "myglwidget.h"

#include <QLabel>
#include <QMainWindow>
#include <QSlider>
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

private slots:
  void on_openfile_triggered();
  void on_averageSimplify_triggered();
  void on_curvSimplify_triggered();
  void on_curve_valueChanged(double arg1);
  void setCell(double);
  void setCurv(float);
  void on_cellLength_valueChanged(double arg1);
  void setStatusMessage(const QString &mess, int timeout = 0);

private:
  Ui::MainWindow *ui;
  MyGLWidget *glwidget;
  QSlider *xSlider;
  QSlider *ySlider;
  QSlider *zSlider;
  QLabel *statusLabel;

  QSlider *createVerticalSlider();
};

#endif // MAINWINDOW_H
