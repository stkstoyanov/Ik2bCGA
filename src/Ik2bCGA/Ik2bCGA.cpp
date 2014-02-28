#include <iostream>
#include <cmath>

struct VectorPx
{
  double x;
  double y;
  double z;
  bool operator==(const VectorPx& _rhs) const
  {
    return _rhs.x == x && _rhs.y == y && _rhs.z == z;
  }
};

struct QuaternionPx
{
  double x;
  double y;
  double z;
  double w;
};

extern "C"
void solveIK(
             const VectorPx& _shoulder,
             const VectorPx& _elbow,
             const VectorPx& _wrist,
             const VectorPx& _goal,
             const VectorPx& _pole,
             QuaternionPx& o_shoulderRot,
             QuaternionPx& o_elbowRot
             /*
             const double _twist,
             */
            )
{
}
