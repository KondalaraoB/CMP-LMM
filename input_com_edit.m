%% function to give input of centre of mass finding problem
function [mT mL r_L0_cTp0 r_L0_cLp0 m0 r_Li1_dash_ci1_pi1 r_Li2_dash_ci2_pi2 r_Li3_dash_ci3_pi3 mi1a mi2a mi3a]=input_com_edit(i)
% to check
%i=2;

mT=1.852;  % mass of trunk body
mL=4.244;   %mass of pay load
r_L0_cTp0=[0 16.74 0]'*1e-3; % distance of trunkbody com from p0
r_L0_cLp0=[0 72.5 64.5]'*1e-3; % distance of pay load com from p0
m0=mT+mL;
% mass of leg components
mi1a=[.442 .442 .442 .442 .442 .442];  % link li1 mass from leg 1 to 6
mi2a=[.117 .117 .117 .117 .117 .117];  % link li2 mass from leg 1 to 6
mi3a=[.303+.002 .303+.002 .303+.002 .303+.002 .303+.002 .303+.002];  % link li2 mass from leg 1 to 6

if(i==2||i==4||i==6)
r_Li1_dash_ci1_pi1=[71.217 -7.28   -14.036]'*1e-3;
r_Li2_dash_ci2_pi2=[71.443 -16.039 -0.186 ]'*1e-3;
r_Li3_dash_ci3_pi3=[99.253 -3.515   0.059 ]'*1e-3;
else
r_Li1_dash_ci1_pi1=[-71.217 -7.28   -14.036]'*1e-3;
r_Li2_dash_ci2_pi2=[-71.443 -16.039 -0.186 ]'*1e-3;
r_Li3_dash_ci3_pi3=[-99.253 -3.515   0.059 ]'*1e-3;   
end






