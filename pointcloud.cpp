#include "pointcloud.h"
#include "colormap.h"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <set>
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
    treeRoot->~Octree();
    //    delete treeRoot;
  }
  treeRoot = new Octree(data, Point<float>(0.0, 0.0, 0.0), miniLength, 2.0);
}

void PointCloud::addColor(std::vector<float> &v) {
  std::vector<float> temp = v;
  v.clear();
  for (int i = 1; i <= temp.size(); i++) {
    v.push_back(temp[i - 1]);
    if (i % 3 == 0) {
      v.push_back(0);
      v.push_back(0);
      v.push_back(0);
    }
  }
}

float PointCloud::distanceOfV3(Point<float> a, Point<float> b) {
  float dx = a.x - b.x;
  float dy = a.y - b.y;
  float dz = a.z - b.z;
  return sqrt(dx * dx + dy * dy + dz * dz);
}

bool PointCloud::cmp(PointCloud::id_curv &a, PointCloud::id_curv &b) {
  return a.curve > b.curve;
}

void PointCloud::matrixSolver(const float mat[], float *answer) {
  float D, D1, D2, D3;
  D = D1 = D2 = D3 = 0.0;

  float md[] = {mat[0 + 0 * 4], mat[1 + 0 * 4], mat[2 + 0 * 4],
                mat[0 + 1 * 4], mat[1 + 1 * 4], mat[2 + 1 * 4],
                mat[0 + 2 * 4], mat[1 + 2 * 4], mat[2 + 2 * 4]};
  float md1[] = {mat[3 + 0 * 4], mat[1 + 0 * 4], mat[2 + 0 * 4],
                 mat[3 + 1 * 4], mat[1 + 1 * 4], mat[2 + 1 * 4],
                 mat[3 + 2 * 4], mat[1 + 2 * 4], mat[2 + 2 * 4]};
  float md2[] = {mat[0 + 0 * 4], mat[3 + 0 * 4], mat[2 + 0 * 4],
                 mat[0 + 1 * 4], mat[3 + 1 * 4], mat[2 + 1 * 4],
                 mat[0 + 2 * 4], mat[3 + 2 * 4], mat[2 + 2 * 4]};
  float md3[] = {mat[0 + 0 * 4], mat[1 + 0 * 4], mat[3 + 0 * 4],
                 mat[0 + 1 * 4], mat[1 + 1 * 4], mat[3 + 1 * 4],
                 mat[0 + 2 * 4], mat[1 + 2 * 4], mat[3 + 2 * 4]};

  D = valueOfMat3(md);
  D1 = valueOfMat3(md1);
  D2 = valueOfMat3(md2);
  D3 = valueOfMat3(md3);

  answer[0] = D1 / D;
  answer[1] = D2 / D;
  answer[2] = D3 / D;
}

float PointCloud::valueOfMat3(const float *mat) {
  return mat[0] * (mat[4] * mat[8] - mat[5] * mat[7]) -
         mat[1] * (mat[3] * mat[8] - mat[5] * mat[6]) +
         mat[2] * (mat[3] * mat[7] - mat[4] * mat[6]);
}

float PointCloud::calculateCure(const std::vector<Point<float>> &points) {
  if (points.size() < 4) { //当前节点的数目不足以进行球面拟合
    return 0.0;
  } else {
    //进行曲面拟合
    int n = points.size(); //样本点的数量
    float sum_x, sum_y, sum_z;
    float average_x, average_y, average_z;
    // u_i = x_i - average_x
    // v_i = y_i - average_y
    // w_i = z_i - average_z
    std::vector<float> u, v, w;

    sum_x = sum_y = sum_z = 0.0;
    for (unsigned int i = 0; i < points.size(); i++) {
      sum_x += points[i].x;
      sum_y += points[i].y;
      sum_z += points[i].z;
    }
    average_x = sum_x / n;
    average_y = sum_y / n;
    average_z = sum_z / n;
    for (unsigned int i = 0; i < points.size(); i += 1) {
      u.push_back(points[i].x - average_x);
      v.push_back(points[i].y - average_y);
      w.push_back(points[i].z - average_z);
    }

    float matrix[12] = {0.0};
    for (int i = 0; i < n; i++) {
      matrix[0 + 0 * 4] += u[i] * u[i];
      matrix[1 + 0 * 4] += u[i] * v[i];
      matrix[2 + 0 * 4] += u[i] * w[i];
      matrix[3 + 0 * 4] +=
          (u[i] * u[i] * u[i] + u[i] * v[i] * v[i] + u[i] * w[i] * w[i]);
      matrix[0 + 1 * 4] += u[i] * v[i];
      matrix[1 + 1 * 4] += v[i] * v[i];
      matrix[2 + 1 * 4] += v[i] * w[i];
      matrix[3 + 1 * 4] +=
          (u[i] * u[i] * v[i] + v[i] * v[i] * v[i] + v[i] * w[i] * w[i]);
      matrix[0 + 2 * 4] += u[i] * w[i];
      matrix[1 + 2 * 4] += w[i] * v[i];
      matrix[2 + 2 * 4] += w[i] * w[i];
      matrix[3 + 2 * 4] +=
          (u[i] * u[i] * w[i] + w[i] * v[i] * v[i] + w[i] * w[i] * w[i]);
    }
    float answerOfMatrix[3];
    matrixSolver(matrix, answerOfMatrix);
    // std::cout<<answerOfMatrix[0]<<'\t'<<answerOfMatrix[1]<<'\t'<<answerOfMatrix[2]<<std::endl;
    float temp_sum = 0.0;
    for (int i = 0; i < n; i++) {
      float du = u[i] - answerOfMatrix[0];
      float dv = v[i] - answerOfMatrix[1];
      float dw = w[i] - answerOfMatrix[2];
      temp_sum += (du * du + dv * dv + dw * dw);
    }
    temp_sum /= n;
    float R = std::sqrt(temp_sum);
    //        std::cout << R << std::endl;
    return 1 / R;
  }
}

PointCloud::PointCloud() { treeRoot = nullptr; }

std::vector<float> PointCloud::averageSimplify(float miniLength) {

  std::cout << "均匀简化" << std::endl;
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
  //加入颜色
  addColor(result);

  return result;
}

std::vector<float> PointCloud::curvatureSimplify() {

  std::vector<Point<float>> cloud;
  for (int i = 0; i < data.size(); i += 3) {
    cloud.push_back(Point<float>(data[i], data[i + 1], data[i + 2]));
  }

  int nearbycount = 10;

  std::vector<std::vector<id_dis>> nearby(
      cloud.size(), std::vector<id_dis>(nearbycount, id_dis(-1, 100000)));
  for (int i = 0; i < cloud.size() - 1; i++) {
    for (int j = i + 1; j < i + 100 && j < cloud.size(); j++) {
      float temp = distanceOfV3(cloud[i], cloud[j]);
      //找到最大值
      int mark = 0;
      float value = nearby[i][0].distance;

      for (int k = 0; k < nearbycount; k++) {
        if (value < nearby[i][k].distance) {
          value = nearby[i][k].distance;
          mark = k;
        }
      }
      if (temp < value) {
        nearby[i][mark].distance = temp;
        nearby[i][mark].id = j;
      }

      mark = 0;
      value = nearby[j][0].distance;
      for (int k = 0; k < nearbycount; k++) {
        if (value < nearby[j][k].distance) {
          value = nearby[j][k].distance;
          mark = k;
        }
      }
      if (temp < value) {
        nearby[j][mark].distance = temp;
        nearby[j][mark].id = i;
      }
      std::cout << "i=" << i << " j=" << j << std ::endl;
    }
  }

  //拟合曲率
  std::vector<id_curv> id_curvs(cloud.size());
  for (int i = 0; i < id_curvs.size(); i++) {
    id_curvs[i].id = i;
    std::vector<Point<float>> nearpoints(nearbycount);
    for (int j = 0; j < nearbycount; j++) {
      nearpoints[j] = cloud[nearby[i][j].id];
    }
    id_curvs[i].curve = calculateCure(nearpoints);
  }

  //排序
  std::sort(std::begin(id_curvs), std::end(id_curvs), cmp);

  //构造返回值
  std::vector<float> result;
  float maxCurve = id_curvs[0].curve;

  for (int i = 0; i < id_curvs.size(); i++) {
    result.push_back(cloud[id_curvs[i].id].x);
    result.push_back(cloud[id_curvs[i].id].y);
    result.push_back(cloud[id_curvs[i].id].z);
    double map = 1 - i / (float)cloud.size();
    colorMap colormap(map);
    result.push_back(colormap.getWormAndColdColorMapRGBResult_R());
    result.push_back(colormap.getWormAndColdColorMapRGBResult_G());
    result.push_back(colormap.getWormAndColdColorMapRGBResult_B());
  }
  return result;
}

std::vector<float> PointCloud::averageSimplifyDBSCAN(float miniLength) {
  std::cout << "聚类简化" << std::endl;
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
      //将节点的聚类平均值加入结果中
      if (temp->data.size() != 0) {
        std::vector<float> jl = temp->getDBSCANPoints();
        for (int i = 0; i < jl.size(); i++) {
          result.push_back(jl[i]);
        }
      }
      q.pop();
    }
    addColor(result);
    return result;
  }
}
