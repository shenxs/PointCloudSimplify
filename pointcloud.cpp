#include "pointcloud.h"
#include <cstring>
PointCloud::PointCloud(std::string path) {
  absMaxCor = 0.0;
  memset(max, -1000000, sizeof(max));
  memset(min, 1000000, sizeof(min));
  int i = 0;

  treeRoot = nullptr;
  std::ifstream modelfile(path);
  if (modelfile.good()) {
    float temp = 0.0;
    while (modelfile >> temp) {
      if (i >= 3)
        i -= 3;
      if (temp > max[i])
        max[i] = temp;
      if (temp < min[i])
        min[i] = temp;
      if (abs(temp) > absMaxCor) {
        absMaxCor = abs(temp);
      }
      data.push_back(temp);
      i++;
    }
    modelfile.close();
  } else {
    std::cerr << "PointCloud read error:\n" << path << std::endl;
  }
  normolize();
}

PointCloud::~PointCloud() {

  if (treeRoot != nullptr) {
    delete treeRoot;
  }
}

void PointCloud::normolize() {
  int it = 0;
  float d[3];
  for (int i = 0; i < 3; i++) {
    d[i] = (max[i] + min[i]) / 2;
  }

  for (auto &i : data) {
    if (it == 3)
      it -= 3;
    i -= d[it];     //中正
    i /= absMaxCor; //压缩到-1，1之间
    it++;
  }
}

void PointCloud::buidTree(float miniLength) {
  if (treeRoot != nullptr) {
    delete treeRoot;
  }
  treeRoot = new Octree(data, Point<float>(0.0, 0.0, 0.0), miniLength, 2.0);
}

PointCloud::PointCloud() { treeRoot = nullptr; }

std::vector<float> PointCloud::averageSimplify(float miniLength) {

  std::vector<float> result;
  if (miniLength >= 1 or miniLength <= 0) {
    return data;
  }

  buidTree(miniLength);
  //开始遍历8叉树简化
  std::queue<Octree *> q;
  Octree *temp = nullptr;
  q.push(treeRoot);
  while (!q.empty()) {
    temp = q.front();
    for (int i = 0; i < 8; i++) {
      if (temp->childs[i] != nullptr) {
        q.push(temp->childs[i]);
      }
    }
    if (temp->data.size() != 0) {
      Point<float> average = temp->getAverageOfPoints();
      result.push_back(average.x);
      result.push_back(average.y);
      result.push_back(average.z);
    }
    q.pop();
  }
  return result;
}

std::vector<float> PointCloud::curvatureSimplify(float miniLength,
                                                 float miniCurva) {
  if (miniLength >= 1 || miniLength <= 0) {
    return data;
  } else {
    std::vector<float> result;
    buidTree(miniLength);
    std::queue<Octree *> q;
    Octree *temp = nullptr;
    q.push(treeRoot);
    while (!q.empty()) {
      temp = q.front();
      for (int i = 0; i < 8; i++) {
        if (temp->childs[i] != nullptr) {
          q.push(temp->childs[i]);
        }
      }
      if (temp->data.size() != 0) {
        float curv = temp->calculateCurvature();
        std::cout << curv << std::endl;
        if (curv > miniCurva) {
          for (unsigned int i = 0; i < temp->data.size(); i++) {
            result.push_back(temp->data[i]);
          }
        } else {
          Point<float> p = temp->getAverageOfPoints();
          result.push_back(p.x);
          result.push_back(p.y);
          result.push_back(p.z);
        }
      }
      q.pop();
    }
    return result;
  }
}
