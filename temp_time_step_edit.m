%function to calculate temporary value of time step



 function[t0s,ta,tb,tc,td,te,t3s]=temp_time_step_edit(HCNC,s0pp,t1minust0,t3minust2)
%    clear all; close all; clc;
%      ii=4; %#ok<NASGU>
%      HCNC=2;
m=4;  %just to call input function

[di1,di2,di3,di,Li,Li1,Li2,Li3,Li3p,sai,phi,d,aplha0_i,...
    r_G_p0_o_i,r_L0_si_p0,...
    gama_r,gama_L,etadot0_i,etadot0_f,eta0_i,etaG_i,CC,h,...
    t0,ts0_i1,etadotG_i,etadotG_f,h_Gi3,thetai10]=inputs_edit();

%i=ii;
%-------------------------------------------------------------------------
%call the time step value for each half cycle..
[ts_3i]=time_step_edit(s0pp,t1minust0,t3minust2);

% call TB time 
[t0,t1,t2,t3]=TB_time_edit(s0pp,t1minust0,t3minust2);

%ta,tb,tc,td,te, t0s,t3s value according to HCNC value

    t0s=t0;
    t3s=ts_3i(HCNC);
    ta=t0s+1/6*(t3s-t0s);
    tb=t0s+2/6*(t3s-t0s);
    tc=t0s+3/3*(t3s-t0s);   % changed from 3/6 to 3/3
    td=t0s+4/6*(t3s-t0s);
    te=t0s+5/6*(t3s-t0s);

end

