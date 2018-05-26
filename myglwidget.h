#ifndef MYGLWIDGET_H
#define MYGLWIDGET_H

#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShader>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>

#include <QOpenGLWidget>
#include <QWidget>

#include <pointcloud.h>
class MyGLWidget : public QOpenGLWidget, protected QOpenGLFunctions {
  Q_OBJECT
public:
  explicit MyGLWidget(QWidget *parent = nullptr);
  ~MyGLWidget();
  void setPointCloud(std::string path);
  void updatedata();

protected:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;
  void keyPressEvent(QKeyEvent *event) override; //处理键盘按下事件
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;

public slots:
  void setXRotation(int angle);
  void setYRotation(int angle);
  void setZRotation(int angle);
  void setScale(float s);
  void averageSimplify(double s = 0.05f);
  void averageSimplifyDBSCAN(double s = 0.05f);
  void curvSimplify(float cellLength = 0.03f, float curve = 30);

signals:
  void xRotationChanged(int angle);
  void yRotationChanged(int angle);
  void zRotationChanged(int angle);
  void celllengthChange(float);
  void curvChange(float);
  void showCurrentStatus(const QString &str, int timeout = 0);

private:
  int m_xRot = 0.0;
  int m_yRot = 0.0;
  int m_zRot = 0.0;
  float m_scale = 1.0f;
  int m_projMatrixLoc;
  int m_modelMatrixLoc;
  int m_viewMatrixLoc;
  QPoint m_lastPos;
  QMatrix4x4 m_proj;
  QMatrix4x4 m_view;
  QMatrix4x4 m_model;
  bool fullscreen; //是否全屏显示
  QOpenGLVertexArrayObject m_vao;
  QOpenGLBuffer m_vbo;
  QOpenGLShader *m_shader;
  QOpenGLShaderProgram *m_program;
  PointCloud *pointcloud;
  std::vector<float> data;
  void updateStateAfterSimplify();
};

#endif // MYGLWIDGET_H
