#include "cga.h"

namespace cga
{

cm::mv<0x01>::type e1 = {1.0};
cm::mv<0x02>::type e2 = {1.0};
cm::mv<0x04>::type e3 = {1.0};
cm::mv<0x08>::type ep = {1.0};
cm::mv<0x10>::type em = {1.0};
cm::mv<0x08, 0x10>::type e0 = 0.5*(em-ep);
cm::mv<0x08, 0x10>::type einf = em+ep;

cb point(const double& _x, const double& _y, const double& _z)
{
  cv v={_x,_y,_z};
  auto p = v+0.5*v*v*einf+e0;
  return p;
}

cb point(const cv& _v)
{
  auto p = _v+0.5*_v*_v*einf+e0;
  return p;
}

cb sphere(
          const double& _x,
          const double& _y, 
          const double& _z,
          const double& _r
         )
{
  cs r = _r;
  cv v={_x,_y,_z};
  auto s = v+0.5*(v*v - r*r)*einf+e0;
  return s;
}

cb sphere(const cv& _v, const double& _r)
{
  cs r = _r;
  auto s = _v+0.5*(_v*_v - r*r)*einf+e0;
  return s;
}

double getX(const cb& _v) { return _v.element<0x01>(); }
double getY(const cb& _v) { return _v.element<0x02>(); }
double getZ(const cb& _v) { return _v.element<0x04>(); }

void display(const cb& _v, const std::string& _signature="")
{
  std::cout << _signature;
  std::cout << getX(_v) << ", ";
  std::cout << getY(_v) << ", ";
  std::cout << getZ(_v);
  std::cout << std::endl;
}

} // cga
