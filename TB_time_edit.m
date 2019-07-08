%function to calculate the trunk body motion time



function [t0,t1,t2,t3]=TB_time_edit(s0pp,t1minust0,t3minust2)
%to check
%clear all; close all; clc;

m=4;%can put any value just to call inputs function
%inputs
[di1,di2,di3,di,Li,Li1,Li2,Li3,Li3p,sai,phi,d,aplha0_i,...
    r_G_p0_o_i,r_L0_si_p0,...
    gama_r,gama_L,etadot0_i,etadot0_f,eta0_i,etaG_i,CC,h,...
    t0,ts0_i1,etadotG_i,etadotG_f,h_Gi3,thetai10]=inputs_edit();

    D_x_i = 0;                           %This is present in time_step_edit & TB_time_edit
    D_x_f = 15*10e-4;                     % in case of change of value need to do it in above also
s=3*s0pp*CC; %total distance travel in half cycle

% time limit given

t1=t1minust0+t0;


% calculation of time at different point i.e. t0,t1,t2,t3
rdot_G_p0_o_f1=(D_x_f);  % stroke along the y- direction for fwd
rdot_G_p0_o_i1=D_x_i;  % stroke along the y- direction for fwd
a_vel=(rdot_G_p0_o_f1-rdot_G_p0_o_i1);
D_del_a=1/t1minust0;
D_del_d=1/t3minust2;


t2minust1=(1/(rdot_G_p0_o_f1))*((s)-((rdot_G_p0_o_i1)+.5*(a_vel))/D_del_a-((rdot_G_p0_o_f1)-.5*(a_vel))/D_del_d);
t2=t2minust1+t1;
t3=t3minust2+t2;

%display([t0,t1,t2,t3])

