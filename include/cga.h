#ifndef _CGA_H_
#define _CGA_H_

#include <string>
#include "gaalet.h"

namespace cga
{

typedef gaalet::algebra<gaalet::signature<4,1>> cm; // conformal model
typedef cm::mv<0x01,0x02,0x04,0x08,0x10>::type cb; // conformal vector
typedef cm::mv<0x01,0x02,0x04>::type cv; // Euclidean vector
typedef cm::mv<0x00>::type cs; // scalar

extern cm::mv<0x01>::type e1;
extern cm::mv<0x02>::type e2;
extern cm::mv<0x04>::type e3;
extern cm::mv<0x08>::type ep;
extern cm::mv<0x10>::type em;
extern cm::mv<0x08, 0x10>::type e0;
extern cm::mv<0x08, 0x10>::type einf;

cb point(const double& _x, const double& _y, const double& _z);
cb point(const cv& _v);
cb sphere(
          const double& _x,
          const double& _y, 
          const double& _z,
          const double& _r
         );
cb sphere(const cv& _v, const double& _r);
cb sphere(const cb& _p, const double& _r);
double getX(const cb& _v);
double getY(const cb& _v);
double getZ(const cb& _v);
void display(const cb& _v, const std::string& _signature);

} // cga

#endif // _CGA_H_
