#ifndef OCTREE_H
#define OCTREE_H

#include "point.h"
#include <math.h>
#include <vector>

class Octree {
private:
  int xit[8] = {1, 1, 1, 1, -1, -1, -1, -1};
  int yit[8] = {1, -1, 1, -1, 1, -1, 1, -1};
  int zit[8] = {1, 1, -1, -1, 1, 1, -1, -1};

  void set8childNull();
  void matrixSolver(const float mat[], float *answer);
  float valueOfMat3(const float mat[]);

public:
  int depth = 0;
  std::vector<float> data;
  //八个象限
  Octree *childs[8];
  Octree();
  Octree(int cellMaxCount, const float pointCloudArry[], float c);
  Octree(const std::vector<float> &pointCloudArry, Point<float> O,
         float miniLength, float currentLength);
  ~Octree();

  Point<float> getAverageOfPoints();
  float calculateCurvature();
};

#endif // OCTREE_H
