#ifndef POINT_H
#define POINT_H

template <class T> class Point {
public:
  T x, y, z;
  Point();
  Point(T x, T y, T z);
  Point(const Point &p);
  Point &operator=(const Point &p);
  ~Point() {}
};

template <class T> Point<T>::Point() { x = y = z = 0; }

template <class T> Point<T>::Point(const Point<T> &p) {
  this->x = p.x;
  this->y = p.y;
  this->z = p.z;
}

template <class T> Point<T> &Point<T>::operator=(const Point<T> &p) {
  this->x = p.x;
  this->y = p.y;
  this->z = p.z;
}

template <class T> Point<T>::Point(T x, T y, T z) {
  this->x = x;
  this->y = y;
  this->z = z;
}

#endif // POINT_H
