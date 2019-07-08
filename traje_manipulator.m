function [ee_manipulator1, Dee_manipulator1]=traje_manipulator(t0, t3, ee_i, ee_f, Dee_i, Dee_f, DDee_i, DDee_f, h)
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
 
 eqn1=subs(x,tt,t0)-ee_i;
 
 eqn2=subs(x,tt,t3)-ee_f;
 
 eqn3=subs(D_x,tt,t0)-Dee_i;
 
 eqn4=subs(D_x,tt,t3)-Dee_f;
 
 eqn5=subs(DD_x,tt,t0)-DDee_i;
 
 eqn6=subs(DD_x,tt,t3)-DDee_f;
 
  
 [A0 A1 A2 A3 A4 A5]=solve(eqn1,eqn2,eqn3,eqn4,eqn5,eqn6,a0,a1,a2,a3,a4,a5) ;
  
 % putting the coefficient value
 x2=A0+A1*tt+A2*tt^2+A3*tt^3+A4*tt^4+A5*tt^5;
 D_x2=diff(x2,tt); 

 x_ei = zeros(1,N1);
 Dx_ei = zeros(1,N1);
 
 for N=1:N1
     n=(N-1)*h;
     ts(N)=n;
     x_ei(N)=subs(x2,tt,ts(N));
     Dx_ei(N)=subs(D_x2,tt,ts(N));
 end
 ee_manipulator1 = x_ei;
 Dee_manipulator1 = Dx_ei;
end