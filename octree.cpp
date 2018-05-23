#include "octree.h"

Octree::Octree() { this->set8childNull(); }

Octree::~Octree() {
  for (int i = 0; i < 8; i++) {
    if (childs[i] != nullptr) {
      delete childs[i];
    }
  }
}

Octree::Octree(const std::vector<float> &pointCloudArry,
               Point<float> orignalPoint, float miniLength,
               float currentLength) {
  cellLength = currentLength;
  if (currentLength <= miniLength) { //递归终止条件
    data = pointCloudArry;
    set8childNull();
  } else { //划分为八个象限
    std::vector<float> temp[8];
    for (unsigned int i = 0; i < pointCloudArry.size(); i += 3) {
      float x, y, z, dx, dy, dz;
      x = pointCloudArry[i];
      y = pointCloudArry[i + 1];
      z = pointCloudArry[i + 2];
      dx = x - orignalPoint.x;
      dy = y - orignalPoint.y;
      dz = z - orignalPoint.z;
      for (int j = 0; j < 8; j++) {
        if (dx * xit[j] >= 0 && dy * yit[j] >= 0 && dz * zit[j] >= 0) {
          temp[j].push_back(x);
          temp[j].push_back(y);
          temp[j].push_back(z);
          break;
        }
      }
    }
    float newCurrentLen = currentLength / 2;
    for (int i = 0; i < 8; i++) {
      if (temp[i].size() != 0) {
        Point<float> newOrg(orignalPoint.x + xit[i] * newCurrentLen / 2,
                            orignalPoint.y + yit[i] * newCurrentLen / 2,
                            orignalPoint.z + zit[i] * newCurrentLen / 2);
        // std::cout<<newOrg.x<<'\t'<<newOrg.y<<'\t'<<newOrg.z<<std::endl;
        childs[i] = new Octree(temp[i], newOrg, miniLength, newCurrentLen);
      } else {
        childs[i] = nullptr;
      }
    }
  }
}

void Octree::set8childNull() {
  for (int i = 0; i < 8; i++) {
    childs[i] = nullptr;
  }
}

Point<float> Octree::getAverageOfPoints() {
  if (this->data.size() != 0) {
    float sum[3] = {0.0, 0.0, 0.0};
    int size = data.size();
    for (int i = 0; i < size; i += 3) {
      for (int j = 0; j < 3; j++) {
        sum[j] += data[i + j];
      }
    }
    for (int i = 0; i < 3; i++) {
      sum[i] /= (size / 3);
    }
    return Point<float>(sum[0], sum[1], sum[2]);

  } else {
    return Point<float>(0, 0, 0);
  }
}

//获取当前节点的聚类平均值
std::vector<float> Octree::getDBSCANPoints() {
  int thereshold = 3;         //一个核心点周围最少的点数
  float eps = cellLength / 4; //核心点最大距离上限

  std::vector<float> result;
  unsigned long size = data.size() / 3;
  if (size < thereshold) {
    return data;
  } else {

    float *distances = new float[size * size]; //每个点之间的距离
    int *count = new int[size];                //满足调节的周围点
    int *flag = new int[size];           //边缘点记录对应的核心点
    float *setSum = new float[size * 3]; //求和
    int *setcount = new int[size];       //计数

    std::vector<int> keyPointList; //关键点
    std::vector<int> sets;

    for (int i = 0; i < size; i++) {
      count[i] = 0;
      flag[i] = -1;
      setSum[i] = setSum[i + 1] = setSum[i + 2] = 0;
      setcount[i] = 0;
    }
    //计算所有点之间的欧式距离
    for (int i = 0; i < size - 1; i++) {
      for (int j = i + 1; j < size; j++) {
        distances[i * size + j] = distanceOfV3(
            Point<float>(data[i * 3], data[i * 3 + 1], data[i * 3 + 2]),
            Point<float>(data[3 * j], data[3 * j + 1], data[3 * j + 2]));
        distances[j * size + i] = distances[i * size + j];
      }
    }

    //找出核心点，边界点，噪点
    for (int i = 0; i < size - 1; i++) {
      for (int j = i + 1; j < size; j++) {
        if (distances[i * size + j] < eps) {
          count[i]++;
          if (count[i] > 0 && count[i] < thereshold) {
            //边界点
            flag[i] = j; //记录边界点
          }
          if (count[i] >= thereshold) { //核心点
            flag[i] = -1;
            keyPointList.push_back(i);
          }

          count[j]++;
          if (count[j] > 0 && count[j] < thereshold) {
            //边界点
            flag[j] = i; //记录边界点
          }
          if (count[j] >= thereshold) {
            flag[j] = -1;
            keyPointList.push_back(j);
          }
        }
      }
    }

    for (int i = 0; i < size; i++) {
      if (count[i] < thereshold) {
        //边界点
        if (count[flag[i]] < thereshold) {
          flag[i] = -1;
        }
      }
    }

    //作并查集
    for (int i = 0; i < keyPointList.size(); i++) {
      bool findRoot = false;
      for (int j = i - 1; j >= 0; j--) {
        if (distances[keyPointList[i] * size + keyPointList[j]] < eps) {
          findRoot = true;
          flag[keyPointList[i]] = flag[keyPointList[j]];
          break;
        }
      }
      if (!findRoot) {
        flag[keyPointList[i]] = keyPointList[i];
        sets.push_back(keyPointList[i]);
      }
    }

    //边缘点指向根结点
    for (int i = 0; i < size; i++) {
      if (flag[i] != -1) {
        int temp = flag[i];
        while (temp != flag[temp]) {
          temp = flag[temp];
        }
        flag[i] = temp;
      }
    }

    for (int i = 0; i < size; i++) {
      if (flag[i] != -1) {
        setcount[flag[i]]++;
        setSum[flag[i]] += data[i];
        setSum[flag[i] + 1] += data[i + 1];
        setSum[flag[i] + 2] += data[i + 2];
      }
    }

    for (int i = 0; i < sets.size(); i++) {
      result.push_back(setSum[sets[i]] / setcount[sets[i]]);
      result.push_back(setSum[sets[i] + 1] / setcount[sets[i]]);
      result.push_back(setSum[sets[i] + 2] / setcount[sets[i]]);
    }

    delete[] setcount;
    delete[] setSum;
    delete[] count;
    delete[] flag;
    delete[] distances;
    return result;
  }
}

//计算当前的节点的曲率
float Octree::calculateCurvature() {
  if (data.size() < 4) { //当前节点的数目不足以进行球面拟合
    return 0.0;
  } else {
    //进行曲面拟合

    int n = data.size() / 3; //样本点的数量
    float sum_x, sum_y, sum_z;
    float average_x, average_y, average_z;
    // u_i = x_i - average_x
    // v_i = y_i - average_y
    // w_i = z_i - average_z
    std::vector<float> u, v, w;

    sum_x = sum_y = sum_z = 0.0;
    for (unsigned int i = 0; i < data.size(); i += 3) {
      sum_x += data[i];
      sum_y += data[i + 1];
      sum_z += data[i + 2];
    }
    average_x = sum_x / n;
    average_y = sum_y / n;
    average_z = sum_z / n;
    for (unsigned int i = 0; i < data.size(); i += 3) {
      u.push_back(data[i] - average_x);
      v.push_back(data[i + 1] - average_y);
      w.push_back(data[i + 2] - average_z);
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
    return 1 / R;
  }
}

float Octree::valueOfMat3(const float *mat) {
  return mat[0] * (mat[4] * mat[8] - mat[5] * mat[7]) -
         mat[1] * (mat[3] * mat[8] - mat[5] * mat[6]) +
         mat[2] * (mat[3] * mat[7] - mat[4] * mat[6]);
}

void Octree::matrixSolver(const float mat[], float *answer) {
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

float Octree::distanceOfV3(Point<float> a, Point<float> b) {
  float dx = a.x - b.x;
  float dy = a.y - b.y;
  float dz = a.z - b.z;
  return sqrt(dx * dx + dy * dy + dz * dz);
}
