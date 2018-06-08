#ifndef COLORMAP_H
#define COLORMAP_H
#include <vector>
class colorMap {
private:
  double x; //需要映射的数值
  double Gauss(double y0, double A, double x0, double w, double x);
  double Circle(double y0, double a, double b, double x0, double x);

public:
  colorMap(double x);
  std::vector<double> getWormAndColdColorMapRGBResult_ALL();
  double getWormAndColdColorMapRGBResult_R();
  double getWormAndColdColorMapRGBResult_G();
  double getWormAndColdColorMapRGBResult_B();

  std::vector<double> getHitMapAll();
  double getHitMapR();
  double getHitMapG();
  double getHitMapB();
};

#endif // COLORMAP_H
