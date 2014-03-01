#include <iostream>
#include <string>
#include <cmath>
#include "cga.h"

struct VectorPx
{
  double x;
  double y;
  double z;

  bool operator==(const VectorPx& _rhs) const
  {
    return _rhs.x == x && _rhs.y == y && _rhs.z == z;
  }

  void show() const
  {
    std::cout << x << ", ";
    std::cout << y << ", ";
    std::cout << z;
    std::cout << std::endl;
  }
};

struct QuaternionPx
{
  double x;
  double y;
  double z;
  double w;

  void show() const
  {
    std::cout << x << ", ";
    std::cout << y << ", ";
    std::cout << z << ", ";
    std::cout << w;
    std::cout << std::endl;
  }
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
  using namespace cga;
  typedef double freal;

  _goal.show();

  if (!(_wrist == _goal))
  {
    cv v_S = {_shoulder.x,_shoulder.y,_shoulder.z};
    cv v_E = {_elbow.x,_elbow.y,_elbow.z};
    cv v_W = {_wrist.x,_wrist.y,_wrist.z};
    cv v_G = {_goal.x,_goal.y,_goal.z};
    
    // compute distances
    cv v_SE = v_E - v_S;
    cv v_EW = v_W - v_E;
    cv v_SG = v_G - v_S;
    cs se_d = magnitude(v_SE); // shoulder-elbow distance
    cs ew_d = magnitude(v_EW); // elbow-wrist distance
    cs sg_d = magnitude(v_SG); // shoulder-goal distance

    auto p_S = point(v_S);
    auto p_E = point(v_E);
    auto p_W = point(v_W);
    auto p_G = point(v_G);

    // compute the elbow point
    auto s_S = sphere(v_S, se_d);
    auto s_G = sphere(v_G, ew_d);
    auto c_E = s_S^s_G; // elbow circle
    auto pl_Swivel = dual(p_S^p_G^e3^einf); // swivel plane
    auto pp_E = pl_Swivel^c_E; // elbow point pair
    // extract the goal elbow point
    auto dpp_E = dual(pp_E);
    // flip the sign on the square root to obtain the other point of the pair
    cs sq = -sqrt(fabs(eval(dpp_E & dpp_E)));
    auto p_EG = (sq - dpp_E) * (!(einf & dpp_E));
    display(p_EG, "Goal Elbow Point: ");

    // compute the YZ shoulder quaternion
    auto pl_Mid = p_E - p_EG;
    auto pl_Elbow = dual(p_S^p_E^p_EG^einf);
    auto l_Elbow = pl_Elbow^pl_Mid;
    auto q_12 = l_Elbow * (!magnitude(l_Elbow)); // normalized middle line
    // compute the X shoulder quaternion
    auto pl_XYR = q_12*e3*(!q_12);
    auto dpl_XYR = dual(pl_XYR);
    auto dpl_Swivel = dual(pl_Swivel);
    auto cos_X = (dpl_XYR & dpl_Swivel) *
                 (!(magnitude(dpl_XYR)*magnitude(dpl_Swivel)));
    //cs cos_X = 1.0;
    freal rcos_X = cos_X.element<0x00>();
    cs sign = pl_XYR & p_G;
    freal rsign = sign / fabs(sign);
    cs cos_QX = rsign * sqrt((1.0 + rcos_X)/2.0);
    cs sin_QX = sqrt((1.0 - rcos_X)/2.0);
    auto q_3 = cos_QX + sin_QX*(e3^e2);
    // compute the XYZ shoulder quaternion
    auto q_S = q_12*q_3;
    // shoulder quaternion output
    o_shoulderRot.x = -q_S.element<0x02 | 0x04>(); 
    o_shoulderRot.y = q_S.element<0x01 | 0x04>();
    o_shoulderRot.z = -q_S.element<0x01 | 0x02>();
    o_shoulderRot.w = q_S.element<0x00>();

    // compute the elbow quaternion
    cs arm_ln = !(se_d*ew_d); // inverse arm length
    // original elbow cosine
    auto l_SEo = dual(p_S^p_E^einf);
    auto l_EWo = dual(p_E^p_W^einf);
    auto dl_SEo = dual(l_SEo);
    auto dl_EWo = dual(l_EWo);
    freal cos_Eo = eval((dl_SEo & dl_EWo)*arm_ln).element<0x00>();
    // goal elbow cosine
    auto l_SEg = dual(p_S^p_EG^einf);
    auto l_EWg = dual(p_EG^p_G^einf);
    auto dl_SEg = dual(l_SEg);
    auto dl_EWg = dual(l_EWg);
    freal cos_Eg = eval((dl_SEg & dl_EWg)*arm_ln).element<0x00>();
    // finding the difference between the original elbow angle and the goal
    freal cos_E = cos(acos(cos_Eg) - acos(cos_Eo));
    // elbow quaternion output
    o_elbowRot.x = 0;
    o_elbowRot.y = -sqrt((1-cos_E)/2.0);
    o_elbowRot.z = 0;
    o_elbowRot.w = sqrt((1+cos_E)/2.0);
  }
  else
  {
    o_shoulderRot.x = 0;
    o_shoulderRot.y = 0;
    o_shoulderRot.z = 0;
    o_shoulderRot.w = 1;

    o_elbowRot.x = 0;
    o_elbowRot.y = 0;
    o_elbowRot.z = 0;
    o_elbowRot.w = 1;
  }
}
