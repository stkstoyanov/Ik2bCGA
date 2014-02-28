#include <iostream>
#include <cmath>
#include "cga.h"

int main()
{
  using namespace cga;

  auto So = sphere(0,0,0,1);
  auto St = sphere(1,0,1,1);
  
  auto C = So^St;
  auto Pl = dual(e0^e1^e3^einf);
  auto Pp = Pl^C;

  auto dPp = dual(Pp);
  // flip the sign on the square root to obtain the other point of the pair
  cs sq = -sqrt(fabs(eval(dPp & dPp)));
  auto P = (sq - dPp) * (!(einf & dPp));
  display(P, "Point: ");
  
  // reflection experiment
  auto P1 = point(1,0,0);
  auto P2 = point(0,0,1);
  auto P3 = point(0,0,-1);

  auto L = dual(P2^P3^einf);
  auto Ln = L * (!magnitude(L));
  P1 = Ln*P1*(!Ln);
  display(P1, "Reflected Point: ");

  std::cout << magnitude(Ln) << std::endl;
}
