#ifndef _PROXY_H_
#define _PROXY_H_

class VectorPx
{
public:
  double x;
  double y;
  double z;

  VectorPx();

  VectorPx(double _x, double _y, double _z);

  bool operator==(const VectorPx& _rhs) const;

  void show() const;
};

class QuaternionPx
{
public:
  double x;
  double y;
  double z;
  double w;

  QuaternionPx();

  QuaternionPx(double _x, double _y, double _z, double _w);

  void show() const;
};

#endif // _PROXY_H_
