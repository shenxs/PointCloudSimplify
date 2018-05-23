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
public:
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
  std::vector<float> curvatureSimplify(float miniLength, float miniCurva);
  std::vector<float> averageSimplifyDBSCAN(float miniLength);
};

#endif // POINTCLOUD_H
