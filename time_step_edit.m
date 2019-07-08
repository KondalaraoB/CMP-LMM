
%program to calculate t3s_1,t3s_2,t3s_3, for each half cycle  
%N number of total cycle, each cycle contains two half cycle
%-----------------------------------------------------------------------
% first to calculate t0,t1,t2,t3 for trunk body for Ncycle...considering
% ramping only at initial and final.... t1-t0=t3-t2=1sec
%list of input used here are
%s,rdot_G_p0_o_i,rdot_G_p0_o_f,s0,

function [ts_3i]=time_step_edit(s0pp,t1minust0,t3minust2)

%to check the program give the artificial input
 %clear all; clc; close all;

 m=4;%can put any value just to call inputs function
%inputs
[di1,di2,di3,di,Li,Li1,Li2,Li3,Li3p,sai,phi,d,aplha0_i,...
    r_G_p0_o_i,r_L0_si_p0,...
    gama_r,gama_L,etadot0_i,etadot0_f,eta0_i,etaG_i,CC,h,...
    t0,ts0_i1,etadotG_i,etadotG_f,h_Gi3,thetai10]=inputs_edit();

    D_x_i = 0;                           %This is present in time_step_edit & TB_time_edit
    D_x_f = 15*10e-4;                     % in case of change of value need to do it in above also
% time limit given
[t0,t1,t2,t3]=TB_time_edit(s0pp,t1minust0,t3minust2);
% calculation of time at different point i.e. t0,t1,t2,t3

D_del_a=1/(t1-t0);
D_del_d=1/(t3-t2);
rdot_G_p0_o_i1=(D_x_i);%## y direction motion
rdot_G_p0_o_f1=(D_x_f);  %## y direction motion
a_vel=(rdot_G_p0_o_f1-rdot_G_p0_o_i1);
%--------------------------------------------------------------------------

% calculate t3s@1,t3s@2,t3s@3

%ts3_1=t1+1/(rdot_G_p0_o_f1)*(6*s0pp-((rdot_G_p0_o_i1)+0.5*(a_vel))*1/D_del_a);% here s0 has to be taken becasue we are calculating its value by compare with tb stroke
%ts3_2=t2-1/((rdot_G_p0_o_f1))*(6*s0pp-((rdot_G_p0_o_i1)+0.5*(a_vel))*1/D_del_d);
ts3_3=t3;

% ts_3i=[ts3_1,ts3_2,ts3_3,ts3_4,ts3_5,ts3_6];
ts_3i=ts3_3;

%display(ts_3i)

