%%
%function to calculate the trajectory of support leg
function [x_ei11 y_ei11 z_ei11 ts11 D_x_ei11 D_y_ei11 D_z_ei11 DD_x_ei11 DD_y_ei11 DD_z_ei11]=full_support_leg(ii,HCNC,ts_start,ts_end)
%to check
% close all; clear all; clc;
% HCNC=3;
% ii=1;
% [ts,ta,tb,tc,td,te,t0s,t3s]=temp_time_step_edit(ii,HCNC);
% ts_start=ts(1);
% ts_end=ts(4);
%inputs
m=4; % just to call the input
[di1,di2,di3,di,Li,Li1,Li2,Li3,Li3p,sai,phi,d,aplha0_i,...
    r_G_p0_o_i,r_L0_si_p0,...
    gama_r,gama_L,etadot0_i,etadot0_f,eta0_i,etaG_i,CC,h,...
    t0,ts0_i1,etadotG_i,etadotG_f,h_Gi3,thetai10]=inputs_edit();
% call input swing parameter
   [Hmi1 del_h h_dash_i3 Href gama_xz_even gama_yz_even gama_xz_ref_even gama_yz_ref_even...
    gama_xz_odd gama_yz_odd gama_xz_ref_odd gama_yz_ref_odd]=input_swing_parameter_edit(HCNC,ii);

% find the range
%N1=ceil((ts_end-ts_start)/h);
x_ei11=zeros(1,ceil((ts_end-ts_start)/h));
y_ei11=zeros(1,ceil((ts_end-ts_start)/h));
z_ei11=zeros(1,ceil((ts_end-ts_start)/h));

D_x_ei11=zeros(1,ceil((ts_end-ts_start)/h));
D_y_ei11=zeros(1,ceil((ts_end-ts_start)/h));
D_z_ei11=zeros(1,ceil((ts_end-ts_start)/h));

DD_x_ei11=zeros(1,ceil((ts_end-ts_start)/h));
DD_y_ei11=zeros(1,ceil((ts_end-ts_start)/h));
DD_z_ei11=zeros(1,ceil((ts_end-ts_start)/h));
ts11=zeros(1,ceil((ts_end-ts_start)/h));
  for j=1:ceil((ts_end-ts_start)/h)
      ts11(j)=(j-1)*h;
  end
