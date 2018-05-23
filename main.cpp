#include "mainwindow.h"
#include "myglwidget.h"
#include <QApplication>

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);

  MainWindow mainwindow;

  mainwindow.show();
  return a.exec();
}
