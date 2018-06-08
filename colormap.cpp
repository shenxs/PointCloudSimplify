#include "colormap.h"
#include <cmath>

double colorMap::Gauss(double y0, double A, double x0, double w, double x) {
  return y0 + A * exp(-(x - x0) * (x - x0) / (2 * w * w));
}

double colorMap::Circle(double y0, double a, double b, double x0, double x) {
  return y0 + b * sqrt(1 - (x - x0) * (x - x0) / (a * a));
}

colorMap::colorMap(double x) { this->x = x; }

std::vector<double> colorMap::getWormAndColdColorMapRGBResult_ALL() {
  std::vector<double> result(3);
  result[0] = Gauss(0.00973, 0.95734, 0.68447, 0.40538, x);
  if (x <= 0.5) {
    result[1] = Gauss(-0.70487, 1.57141, 0.51782, 0.54700, x);
  } else {
    result[1] = Circle(-0.97384, 0.96412, 1.96264, 0.17749, x);
  }
  result[2] = Gauss(-0.05837, 1.05992, 0.28797, 0.39754, x);

  return result;
}

double colorMap::getWormAndColdColorMapRGBResult_R() {
  return Gauss(0.00973, 0.95734, 0.68447, 0.40538, x);
}

double colorMap::getWormAndColdColorMapRGBResult_G() {
  if (x <= 0.5) {
    return Gauss(-0.70487, 1.57141, 0.51782, 0.54700, x);
  } else {
    return Circle(-0.97384, 0.96412, 1.96264, 0.17749, x);
  }
}

double colorMap::getWormAndColdColorMapRGBResult_B() {
  return Gauss(-0.05837, 1.05992, 0.28797, 0.39754, x);
}

std::vector<double> colorMap::getHitMapAll() {
  std::vector<double> rgb(3);
  if (x > 0.75) {
    rgb = {1, 0, 0};
  } else if (x > 0.5) {
    rgb = {0.75, 1, 0};
  } else if (x > 0.25) {
    rgb = {0, 1, 0};
  } else {
    rgb = {0, 0, 1};
  }
}
