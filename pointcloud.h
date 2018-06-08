#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "octree.h"
#include "point.h"
#include <fstream>
#include <iostream>
#include <math.h>
#include <queue>
#include <string>
#include <vector>

class PointCloud {
private:
  void buidTree(float miniLength); //创建最小长度是miniLength的八叉树
  void addColor(std::vector<float> &);
  float distanceOfV3(Point<float> a, Point<float> b);
  struct id_curv {
    int id;
    float curve;
  };
  struct id_dis {
    id_dis(int i, float d) {
      id = i;
      distance = d;
    }
    int id;
    float distance;
  };
  static bool cmp(id_curv &a, id_curv &b);

public:
  float calculateCure(const std::vector<Point<float>> &points);
  void matrixSolver(const float mat[], float *answer);
  float valueOfMat3(const float mat[]);
  std::vector<float> data;
  float absMaxCor;  //点云的坐标边界，绝对值最大
  float max[3];     //记录xyz的最大值
  float min[3];     //同上
  Octree *treeRoot; //八叉树根节点

  PointCloud();
  PointCloud(std::string path); //从文件读取点云数据
  ~PointCloud();

  void normolize(); //将数据标准化，映射到-1～1之间
  std::vector<float> averageSimplify(float miniLength);
  std::vector<float> curvatureSimplify();
  std::vector<float> averageSimplifyDBSCAN(float miniLength);
};

#endif // POINTCLOUD_H
