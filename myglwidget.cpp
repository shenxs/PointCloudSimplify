#include "myglwidget.h"
#include <GL/glu.h>
#include <QCoreApplication>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <iostream>
#include <math.h>
#include <string>

MyGLWidget::MyGLWidget(QWidget *parent) : QOpenGLWidget(parent) {
  fullscreen = false;
}

MyGLWidget::~MyGLWidget() {
  delete pointcloud;
  makeCurrent();
  delete m_program;
  m_vbo.destroy();
  m_vao.destroy();
  doneCurrent();
}

static void qNormalizeAngle(int &angle) {
  while (angle < 0)
    angle += 360 * 16;
  while (angle > 360 * 16)
    angle -= 360 * 16;
}

void MyGLWidget::initializeGL() //此处开始对OpenGL进行所以设置
{
  initializeOpenGLFunctions();
  m_program = new QOpenGLShaderProgram(this);
  m_program->addShaderFromSourceFile(
      QOpenGLShader::Vertex,
      "/home/richard/Qt Project/PointCloudSimplify/shader.vert");
  m_program->addShaderFromSourceFile(
      QOpenGLShader::Fragment,
      "/home/richard/Qt Project/PointCloudSimplify/shader.frag");
  m_program->link();
  m_program->bind();

  m_projMatrixLoc = m_program->uniformLocation("projMatrix");
  m_modelMatrixLoc = m_program->uniformLocation("modelMatrix");
  m_viewMatrixLoc = m_program->uniformLocation("viewMatrix");

  pointcloud = new PointCloud();
  updatedata();

  QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
  f->glEnableVertexAttribArray(0);
  f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
  glPointSize(1);

  m_view.setToIdentity();
  m_proj.setToIdentity();
  m_view.translate(0, 0, -1);
  m_model.setToIdentity();
  m_program->release();
}

void MyGLWidget::resizeGL(int w, int h) //重置OpenGL窗口的大小
{

  m_proj.setToIdentity();
  //  m_proj.perspective(45.0f, GLfloat(w) / h, 0.01f, 10.0f);
  float radious = GLfloat(w) / h;
  m_proj.ortho(-1 * radious, 1 * radious, -1, 1, -10.0f, 10.0f);
}

void MyGLWidget::paintGL() //从这里开始进行所以的绘制
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);

  glClearColor(1.0, 1.0, 1.0, 1.0);

  //  m_proj.setToIdentity();
  m_model.setToIdentity();
  //  m_view.setToIdentity();

  m_model.rotate(m_yRot / 16, 1, 0, 0);
  m_model.rotate(m_xRot / 16, 0, 1, 0);
  m_model.rotate(m_zRot / 16.0f, 0, 0, 1);

  m_model.scale(m_scale);

  m_program->bind();

  m_program->setUniformValue(m_modelMatrixLoc, m_model);
  m_program->setUniformValue(m_viewMatrixLoc, m_view);
  m_program->setUniformValue(m_projMatrixLoc, m_proj);

  glDrawArrays(GL_POINTS, 0, data.size() / 3);

  m_program->release();
}

void MyGLWidget::keyPressEvent(QKeyEvent *event) {}

void MyGLWidget::mousePressEvent(QMouseEvent *event) {
  m_lastPos = event->pos();
}

void MyGLWidget::mouseMoveEvent(QMouseEvent *event) {
  int dx = event->x() - m_lastPos.x();
  int dy = event->y() - m_lastPos.y();

  if (event->buttons() & Qt::LeftButton) {
    setXRotation(m_xRot + dx);
    setYRotation(m_yRot + dy);

  } else if (event->buttons() & Qt::RightButton) {
    setXRotation(m_xRot + dy);
    setZRotation(m_zRot + dx);
  }
}

void MyGLWidget::wheelEvent(QWheelEvent *event) {
  if (event->angleDelta().y() > 0) {
    m_scale *= 2;
    update();
  } else if (event->angleDelta().y() < 0) {
    m_scale *= 0.5;
    update();
  }
  std::cout << m_scale << std::endl;
}

void MyGLWidget::setXRotation(int angle) {
  qNormalizeAngle(angle);
  if (angle != m_xRot) {
    m_xRot = angle;
    emit xRotationChanged(angle);
    update();
  }
}

void MyGLWidget::setYRotation(int angle) {
  qNormalizeAngle(angle);
  if (angle != m_yRot) {
    m_yRot = angle;
    emit yRotationChanged(angle);
    update();
  }
}

void MyGLWidget::setZRotation(int angle) {
  qNormalizeAngle(angle);
  if (angle != m_zRot) {
    m_zRot = angle;
    emit zRotationChanged(angle);
    update();
  }
}

void MyGLWidget::setScale(float s) {
  if (s != m_scale) {
    m_scale = s;
  }
}

void MyGLWidget::setPointCloud(std::string path) {
  if (path != "") {
    if (pointcloud != nullptr) {
      delete pointcloud;
    }
    pointcloud = new PointCloud(std::string(path));
  }
  data = pointcloud->data;
  std::string str("当前的点云模型包含" + std::to_string(data.size() / 3) +
                  "个点");
  QString currentState = QString::fromStdString(str);
  emit showCurrentStatus(currentState);
  updatedata();
}

void MyGLWidget::updatedata() {
  if (!m_vao.isCreated())
    m_vao.create();
  if (m_vao.isCreated())
    m_vao.bind();

  m_vbo.create();
  m_vbo.bind();
  m_vbo.allocate(&data[0], data.size() * sizeof(float));
  update();
}

void MyGLWidget::averageSimplify(double cellLength) {
  data = pointcloud->averageSimplify((float)cellLength);
  emit celllengthChange((float)cellLength);
  updatedata();
  updateStateAfterSimplify();
}

void MyGLWidget::updateStateAfterSimplify() {
  std::string str("点云模型：" + std::to_string(pointcloud->data.size() / 3) +
                  "点，" + "简化后：" + std::to_string(data.size() / 3) + "点");
  QString currentState = QString::fromStdString(str);
  emit showCurrentStatus(currentState);
}

void MyGLWidget::curvSimplify(float cellLength, float curve) {
  data = pointcloud->curvatureSimplify(cellLength, curve);
  emit celllengthChange((float)cellLength);
  emit curvChange(curve);
  updatedata();
  updateStateAfterSimplify();
}
