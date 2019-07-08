function [th_manipulator1] = angle_manipulator(t0, t3, th_i, th_f, Dth_i, Dth_f, DDth_i, DDth_f, h)
%%% to check
% t0 = 0;
% t3 = 2.4;
% th_i = 10;
% th_f = 15;
% Dth_i = 0;
% Dth_f = 0;
% DDth_i = 0;
% DDth_f = 0;
% h=0.02;

syms tt a0 a1 a2 a3 a4 a5

N1 = ceil(t3/h);

 x=a0+a1*tt+a2*tt^2+a3*tt^3+a4*tt^4+a5*tt^5;
 D_x=diff(x,tt);
 DD_x=diff(D_x,tt); 
 
 %6 equation for to solve for coefficient
 
 eqn1=subs(x,tt,t0)-th_i;
 
 eqn2=subs(x,tt,t3)-th_f;
 
 eqn3=subs(D_x,tt,t0)-Dth_i;
 
 eqn4=subs(D_x,tt,t3)-Dth_f;
 
 eqn5=subs(DD_x,tt,t0)-DDth_i;
 
 eqn6=subs(DD_x,tt,t3)-DDth_f;
 
  
 [A0 A1 A2 A3 A4 A5]=solve(eqn1,eqn2,eqn3,eqn4,eqn5,eqn6,a0,a1,a2,a3,a4,a5) ;
  
 % putting the coefficient value
 x2=A0+A1*tt+A2*tt^2+A3*tt^3+A4*tt^4+A5*tt^5;

 x_ei=zeros(1,N1);
 
 
 for N=1:N1
     n=(N-1)*h;
     ts(N)=n;
     x_ei(N)=subs(x2,tt,ts(N));
 end
 th_manipulator1 = x_ei;
end