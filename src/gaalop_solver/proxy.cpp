#include <iostream>
#include "proxy.h"

VectorPx::VectorPx(){;}

VectorPx::VectorPx(double _x, double _y, double _z): x(_x), y(_y), z(_z) {;}

bool VectorPx::operator==(const VectorPx& _rhs) const
{
  return _rhs.x == x && _rhs.y == y && _rhs.z == z;
}

void VectorPx::show() const
{
  std::cout << x << ", ";
  std::cout << y << ", ";
  std::cout << z;
  std::cout << std::endl;
}

QuaternionPx::QuaternionPx(){;}

QuaternionPx::QuaternionPx(double _x, double _y, double _z, double _w):
  x(_x), y(_y), z(_z), w(_w) {;}

void QuaternionPx::show() const
{
  std::cout << x << ", ";
  std::cout << y << ", ";
  std::cout << z << ", ";
  std::cout << w;
  std::cout << std::endl;
}
