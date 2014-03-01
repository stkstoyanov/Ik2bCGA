#include <iostream>
#include <cmath>
#include "cga.h"

int main()
{
  using namespace cga;

  /*
  // elbow experiment
  cv v_S = {0,0,0};
  auto p_S = point(v_S);
  auto s_S = sphere(v_S, 1);

  cv v_G = {1,0.5,1};
  auto p_G = point(v_G);
  auto s_G = sphere(v_G, 1);

  // find the goal elbow circle
  auto c_E = s_S^s_G;
  auto pl_Swivel = dual(p_S^p_G^e3^einf);
  auto pp_E = pl_Swivel^c_E;

  // find the goal elbow position
  auto dpp_E = dual(pp_E);
  // flip the sign on the square root to obtain the other point of the pair
  cs sq = -sqrt(fabs(eval(dpp_E & dpp_E)));
  auto p_E = (sq - dpp_E) * (!(einf & dpp_E));
  display(p_E, "Point: ");
  */
  
  /*
  // reflection experiment
  auto P1 = point(1,0,0);
  auto P2 = point(0,0,1);
  auto P3 = point(0,0,-1);
  auto L = dual(P2^P3^einf);
  auto Ln = L * (!magnitude(L));
  P1 = Ln*P1*(!Ln);
  display(P1, "Reflected Point: ");
  std::cout << magnitude(Ln) << std::endl;
  */

  /*
  // scalar output test
  cv v_T = {2,0,0};
  std::cout << sqrt(fabs(eval(v_T*v_T).element<0x00>())) << std::endl;
  std::cout << magnitude(v_T) << std::endl;
  */

  /*
  // extracting blades
  auto t = 21.0*(e1^e2);
  std::cout << t.element<0x01 | 0x02>() << std::endl;
  */
}
