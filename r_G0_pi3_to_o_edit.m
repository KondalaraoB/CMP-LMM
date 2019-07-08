% programme to calculate trajectory when trunk body moves in slant surace

function[r_G0_pi3_oa,gama,r_G_pi3_oa,D_r_G_pi3_oa,DD_r_G_pi3_oa]=r_G0_pi3_to_o_edit(m,i,s0pp,t1minust0,t3minust2,del_t12,theta_c,sw,theta1na,beta2na,beta3na,position_ini,szi)
%to check
% clear all; close all; clc;
% i=6;

%inputs
    %m=4;                     %can put any value just to call inputs function
   [di1,di2,di3,di,Li,Li1,Li2,Li3,Li3p,sai,phi,d,aplha0_i,...
    r_G_p0_o_i,r_L0_si_p0,...
    gama_r,gama_L,etadot0_i,etadot0_f,eta0_i,etaG_i,CC,h,...
    t0,ts0_i1,etadotG_i,etadotG_f,h_Gi3,thetai10]=inputs_edit();

%calling r_G_pi3_to_o function which is to be transform here

rem_szi = (rem(szi,2)==0);
if rem_szi==0
[r_G_pi3_oa,D_r_G_pi3_oa,DD_r_G_pi3_oa,gama,NN1]=r_G_pi3_to_o_145support(i,s0pp,t1minust0,t3minust2,del_t12,theta_c,sw,theta1na,beta2na,beta3na,position_ini); % m is input here just to correct function, m has no use 
else 
[r_G_pi3_oa,D_r_G_pi3_oa,DD_r_G_pi3_oa,gama,NN1]=r_G_pi3_to_o_236support(i,s0pp,t1minust0,t3minust2,del_t12,theta_c,sw,theta1na,beta2na,beta3na,position_ini); 
end
% to call the transform matrix
etaG_t=etaG_i;
eta0_t=eta0_i; %############# since it is not used here so i simply gave the constant value onlY A_G_G0 is used in  r_G0_pi3_oa(N,:)=(A_G0_G*r_G_pi3_o)';
[A_G_L0,A_G_G0]=transfm_edit();
A_G0_G=A_G_G0';
% iteration to rotate the frame by A_G_G0
r_G0_pi3_oa=zeros(NN1,3);
tme=zeros(NN1);
x_G0_ei=zeros(NN1);
y_G0_ei=zeros(NN1);
z_G0_ei=zeros(NN1);
for N=1:NN1
    tme(N)=(N-1)*h;
    r_G_pi3_o=r_G_pi3_oa(N,:)';
    r_G0_pi3_oa(N,:)=(A_G0_G*r_G_pi3_o)';
    x_G0_ei(N)=r_G0_pi3_oa(N,1);
    y_G0_ei(N)=r_G0_pi3_oa(N,2);
    z_G0_ei(N)=r_G0_pi3_oa(N,3);
end
