clear;
clc;
syms xim1_x xim1_y xix xiy xip1_x xip1_y;
xim1 = [xim1_x, xim1_y];
xi = [xix, xiy];
xip1 = [xip1_x, xip1_y];

Dxi = xi - xim1;
Dxip1 = xip1 - xi;

absDxi = sqrt(sum(Dxi.*Dxi));
absDxip1 = sqrt(sum(Dxip1.*Dxip1));

Dphi = acos((Dxi*Dxip1')/(absDxi*absDxip1));

% if (-Dxip1(1)*Dxi(2)+Dxip1(2)*Dxi(1) < 0)
%     Dphi = -Dphi;
% end

f = power(Dphi/absDxi - 1/0.6 , 2);

grad1 = [diff(f,xix), diff(f,xiy)];

%%%%%%%%%%%%%%%%%%%%%%%%%point
xim1_x = 0; xim1_y = 0;
xix = 0; xiy = 1;
xip1_x = -0.8; xip1_y = 1.5;

xim1 = eval(xim1); xi = eval(xi); xip1 = eval(xip1);
Dxi = eval(Dxi); Dxip1 = eval(Dxip1); absDxi = eval(absDxi); absDxip1 = eval(absDxip1);
Dphi = eval(Dphi);
grad1 = eval(grad1)

%%%%%%%%%%%%%%%%%%%%compute
absDxiInv = 1 / absDxi;
absDxip1Inv = 1/absDxip1;
msinDphiInv = -1/sin(Dphi);
u = absDxiInv * msinDphiInv;
v = Dphi*absDxiInv*absDxiInv;
p1 = absDxiInv*absDxip1Inv*(xip1 - 2*xi + xim1) - ...
           Dxi*Dxip1'*absDxiInv*absDxiInv*absDxip1Inv*absDxip1Inv*( ...
             absDxip1*absDxiInv*(xi - xim1) + absDxi*absDxip1Inv*(xi - xip1));
p2 = absDxiInv*(xi - xim1);
grad2 = 2*(Dphi*absDxiInv - 1/0.6)*(u*p1 - v*p2)