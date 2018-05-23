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

  float cellLength;

  void set8childNull();
  void matrixSolver(const float mat[], float *answer);
  float valueOfMat3(const float mat[]);
  float distanceOfV3(Point<float> a, Point<float> b);

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
  std::vector<float> getDBSCANPoints();
  float calculateCurvature();
};

#endif // OCTREE_H
