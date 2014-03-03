#include <iostream>
#include <cmath>
#include "cga.h"

using namespace std;

int main()
{
  using namespace cga;

  // elbow experiment
  double radius = 1;
  cv v_S = {0,0,0};
  auto p_S = point(v_S);

  cv v_G = {1,0.5,1};
  auto p_G = point(v_G);
  auto p_E = point(radius,0,0);
  auto p_P = point(0,0,1);
  auto p_Y = point(1,0,0);

  // find the goal elbow circle
  auto s_S = sphere(v_S, radius);
  auto s_G = sphere(v_G, radius);
  auto c_E = s_S^s_G;
  auto l_G = dual(p_S^p_G^einf);

  auto pl_Swivel = dual(p_P^p_G^p_S^einf);
  cv v_Swivel = {pl_Swivel.element<0x01>(),
                 pl_Swivel.element<0x02>(),
                 pl_Swivel.element<0x04>()};
  v_Swivel = v_Swivel*(!magnitude(v_Swivel));
  std::cout << v_Swivel << std::endl;

  auto pl_YZ = dual(e2^p_E^p_S^einf);
  cv v_YZ = {pl_YZ.element<0x01>(),
             pl_YZ.element<0x02>(),
             pl_YZ.element<0x04>()};
  v_YZ = v_YZ*(!magnitude(v_YZ));
  std::cout << v_YZ << std::endl;
  
  cs angle = v_Swivel & v_YZ;
  auto norm = dual(pl_Swivel);
  cv n = {-norm.element<0x02 | 0x04>(),
    norm.element<0x01 | 0x04>(),
    -norm.element<0x01 | 0x02>()};
  cout << norm << endl;
  cout << n << endl;

  auto pp_E = dual(c_E^pl_Swivel);
  // flip the sign on the square root to obtain the other point of the pair
  cs sq_E = sqrt(fabs((pp_E & pp_E).element<0x00>()));
  auto p_EG = (sq_E - pp_E) * (!(einf & pp_E));
  display(p_EG, "Elbow: ");

  /*
  // reflection experiment
  cv P1 = {1,0,0};
  auto P2 = point(0,0,1);
  auto P3 = point(0,0,-1);
  auto L = dual(P2^P3^einf);
  auto Ln = L * (!magnitude(L));
  auto P = Ln*P1*(!Ln);
  display(P, "Reflected Point: ");
  cout << magnitude(Ln) << endl;
  */

  /*
  // scalar output test
  cv v_T = {2,0,0};
  cout << sqrt(fabs(eval(v_T*v_T).element<0x00>())) << endl;
  cout << magnitude(v_T) << endl;
  */

  /*
  // extracting blades
  auto t = 21.0*(e1^e2);
  cout << t.element<0x01 | 0x02>() << endl;
  */

  /*
  // plane magnitude tests
  auto P1 = (point(0,0,2));
  auto P2 = (point(0,0,-2));
  auto P3 = (point(0,1,0));
  auto pl_T = (dual(P1^P2^P3^einf));
  cout << magnitude(pl_T).element<0x00>() << endl; 
  */
}
