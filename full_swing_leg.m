%%
%function to calculate the full swing leg trajectory
   function [x_ei11 y_ei11 z_ei11 ts11 D_x_ei11 D_y_ei11 D_z_ei11 DD_x_ei11 DD_y_ei11 DD_z_ei11]=full_swing_leg(ii,HCNC,ts1_start,ts1_end,del_t12,sw)
%%%to check
% clc; clear all; close all;
% HCNC=1;
% ii=4;
% [t0s,ta,tb,tc,td,te,t3s]=temp_time_step_edit(HCNC);
% ts1_start=ta;
% ts1_end=tc;
%%
%inputs1
i=ii;
m=4; % just to call the input
[di1,di2,di3,di,Li,Li1,Li2,Li3,Li3p,sai,phi,d,aplha0_i,...
    r_G_p0_o_i,r_L0_si_p0,...
    gama_r,gama_L,etadot0_i,etadot0_f,eta0_i,etaG_i,CC,h,...
    t0,ts0_i1,etadotG_i,etadotG_f,h_Gi3,thetai10]=inputs_edit();
% call input swing parameter
   [Hmi1 del_h h_dash_i3 Href gama_xz_even gama_yz_even gama_xz_ref_even gama_yz_ref_even...
    gama_xz_odd gama_yz_odd gama_xz_ref_odd gama_yz_ref_odd]=input_swing_parameter_edit(HCNC,ii);

%%
% calculate the input
%solving for local frame i.e. taking p_i3 as local point (0 0 0)
  x_L_pi3=0;
  y_L_pi3=0;
  z_L_pi3=0;
 p_L_pi3=[x_L_pi3 y_L_pi3 z_L_pi3]'; %#ok<NASGU>
  
  %converting p_dash_i3 to local basis at p_i3
  x_L_p_dash_i3=x_L_pi3+sw;
  y_L_p_dash_i3=y_L_pi3;
  z_L_p_dash_i3=h_dash_i3;
  p_L_dash_i3=[x_L_p_dash_i3 y_L_p_dash_i3 z_L_p_dash_i3]'; %#ok<NASGU>
  
  %converting Q_i3 to local p_i3 basis
  % %for Qi3
     z_L_Qi3=Hmi1+del_h;
     del_z_pi3_Qi3=Hmi1+del_h;
     if(i==2||i==4||i==6)
         gama_xz=gama_xz_even;
         gama_yz=gama_yz_even;
         gama_xz_ref=gama_xz_ref_even; 
         gama_yz_ref=gama_yz_ref_even; %#ok<NASGU>
     else
         gama_xz=gama_xz_odd;
         gama_yz=gama_yz_odd;
         gama_xz_ref=gama_xz_ref_odd; 
         gama_yz_ref=gama_yz_ref_odd; %#ok<NASGU>
     end
         
     x_L_Qi3=x_L_pi3+del_z_pi3_Qi3*tand(gama_xz);
     y_L_Qi3=y_L_pi3+del_z_pi3_Qi3*tand(gama_yz);
     
     Q_i3=[x_L_Qi3 y_L_Qi3 z_L_Qi3]'; %#ok<NASGU>
  
  %converting T_i3 to local  p_i3 basis
  % for p_dash_i3
     x_L_p_dash_i3=x_L_pi3+sw;
     y_L_p_dash_i3=y_L_pi3;    %for one complete movement of swing leg, distance travel is 2*s0
     z_L_p_dash_i3=h_dash_i3;
     p_dash_i3=[x_L_p_dash_i3 y_L_p_dash_i3 z_L_p_dash_i3]'; %#ok<NASGU>
 
     
     %for Ti3
   del_z_p_dash_i3_T_i3=Hmi1+del_h-h_dash_i3;
   gama_dash_xz=gama_xz+(gama_xz_ref-gama_xz)*h_dash_i3/Href;
   gama_dash_yz=gama_yz+(gama_xz_ref-gama_xz)*h_dash_i3/Href;
   x_L_Ti3=x_L_p_dash_i3+del_z_p_dash_i3_T_i3*tand(gama_dash_xz);
   y_L_Ti3=0;
   z_L_Ti3=z_L_Qi3;
   T_i3=[x_L_Ti3 y_L_Ti3 z_L_Ti3]'; %#ok<NASGU>
   
   %%
    % calling the time step

      t_s0=ts1_start;
      t_s3=ts1_end;
      t_s1=t_s0+del_t12;
      t_s2=t_s3-del_t12;
      t_s=[t_s0 t_s1 t_s2 t_s3];
 
   
   %%
   syms t
  t0s=t_s(1,1)-t_s(1,1);
  t1s=t_s(1,2)-t_s(1,1);
  t2s=t_s(1,3)-t_s(1,1);
  t3s=t_s(1,4)-t_s(1,1);
 
  N1=ceil((ts1_end-ts1_start)/h);
  del_pQ=(t-t0s)/(t1s-t0s);
  
  del_Tp_dash=(t3s-t)/(t3s-t2s);

%----------------------------------------------------------------------------
%trajectory along z
syms t
  z_ei_t0s=z_L_pi3;
  z_ei_t1s=z_L_Qi3;
  z_ei_t2s=z_L_Ti3; %#ok<NASGU>
  z_ei_t3s=z_L_p_dash_i3;
  a_zi_pQ=z_L_Qi3-z_L_pi3;
  a_zi_Tp_dash=z_L_p_dash_i3-z_L_Ti3;

  
  %to difff to find acc. and velovity for time t0 to t1
  z_e1=z_ei_t0s+a_zi_pQ*(del_pQ)^2*(3-2*del_pQ);%to difff to find acc. and velovity
  D_z_e1=diff(z_e1,t);
  DD_z_e1=diff(D_z_e1,t);
  
  %to difff to find acc. and velovity for time t1 to t3
  z_e2=z_ei_t1s;
  D_z_e2=diff(z_e2,t);
  DD_z_e2=diff(D_z_e2,t);
  
  %to difff to find acc. and velovity for time t2 to t3
  z_e3=z_ei_t3s-a_zi_Tp_dash*del_Tp_dash^2*(3-2*del_Tp_dash);
  D_z_e3=diff(z_e3,t);
  DD_z_e3=diff(D_z_e3,t);
  
  z_ei=zeros(1,N1);
  D_z_ei=zeros(1,N1);
  DD_z_ei=zeros(1,N1);
  
  ts1=zeros(1,N1);
  
  for N=1:N1
      n=(N-1)*h;
      ts1(N)=n;
      
      if (ts1(N)<=t0s||ts1(N)<=t1s)
          z_ei(N)=subs(z_e1,t,ts1(N));
          z_ei(N)=vpa(z_ei(N));
          D_z_ei(N)=subs(D_z_e1,t,ts1(N));
          D_z_ei(N)=vpa(D_z_ei(N));
          DD_z_ei(N)=subs(DD_z_e1,t,ts1(N));
          DD_z_ei(N)=vpa(DD_z_ei(N));
          
      elseif(ts1(N)<=t1s||ts1(N)<=t2s)
         z_ei(N)=subs(z_e2,t,ts1(N));
          z_ei(N)=vpa(z_ei(N));
          D_z_ei(N)=subs(D_z_e2,t,ts1(N));
          D_z_ei(N)=vpa(D_z_ei(N));
          DD_z_ei(N)=subs(DD_z_e2,t,ts1(N));
          DD_z_ei(N)=vpa(DD_z_ei(N));
      elseif(ts1(N)<=t2s||ts1(N)<=t3s);
          z_ei(N)=subs(z_e3,t,ts1(N));
          z_ei(N)=vpa(z_ei(N));
          D_z_ei(N)=subs(D_z_e3,t,ts1(N));
          D_z_ei(N)=vpa(D_z_ei(N));
          DD_z_ei(N)=subs(DD_z_e3,t,ts1(N));
          DD_z_ei(N)=vpa(DD_z_ei(N));
      end
  end
  z_ei(N)=z_L_p_dash_i3;   %at the end the point have to reach pi3_dash
%   D_z_ei(N)=0;   %at the end the point have to reach pi3_dash
%   DD_z_ei(N)=0;   %at the end the point have to reach pi3_dash
  
  z_ei11=z_ei(1,ceil((ts1_start-ts1_start)/h+1):ceil((ts1_end-ts1_start)/h));
  D_z_ei11=D_z_ei(1,ceil((ts1_start-ts1_start)/h+1):ceil((ts1_end-ts1_start)/h));
  DD_z_ei11=DD_z_ei(1,ceil((ts1_start-ts1_start)/h+1):ceil((ts1_end-ts1_start)/h));
  
  ts_11=ts1(1,ceil((ts1_start-ts1_start)/h+1):ceil((ts1_end-ts1_start)/h)); %ts11(ceil(N1/2)); % find element wise substraction
  ts11=zeros(1,ceil((ts1_end-ts1_start)/h));
  for j=1:ceil((ts1_end-ts1_start)/h)
      ts11(j)=ts_11(j)-ts_11(1);
      
      
  end
%   subplot(2,2,3)
%   plot(ts11,z_ei11)
%   title('trajectory along z')
 
  
 %-------------------------------------------------------------------------
  %trajectory along y axis

  
  y_ei=zeros(1,N1);
  D_y_ei=zeros(1,N1);
  DD_y_ei=zeros(1,N1);
  for N=1:N1
      n=(N-1)*h;
      ts1(N)=n;
      
      %y_ei(N)=subs(y_ei1,t,ts1(N));
      y_ei(N)=0;%vpa(subs(y_ei(N)));
      %D_y_ei(N)=subs(D_y_ei1,t,ts1(N));
      D_y_ei(N)=0;%vpa(subs(D_y_ei(N)));
      %DD_y_ei(N)=subs(DD_y_ei1,t,ts1(N));
      DD_y_ei(N)=0;%vpa(subs(DD_y_ei(N)));
  end
%y_ei(N)=y_L_p_dash_i3;   %at the end the point have to reach pi3_dash
% D_y_ei(N)=0;   %at the end the point have to reach pi3_dash
% DD_y_ei(N)=0;   %at the end the point have to reach pi3_dash
  
y_ei11=y_ei(1,ceil((ts1_start-ts1_start)/h+1):ceil((ts1_end-ts1_start)/h));
D_y_ei11=D_y_ei(1,ceil((ts1_start-ts1_start)/h+1):ceil((ts1_end-ts1_start)/h));
DD_y_ei11=DD_y_ei(1,ceil((ts1_start-ts1_start)/h+1):ceil((ts1_end-ts1_start)/h));

% subplot(2,2,2)
% plot(ts11,y_ei11)
% title('trajectory along y')

%-------------------------------------------------------------------------------  
  %trajectory along x axis
  syms t
  x_ei_t0s=x_L_pi3;
  x_ei_t3s=x_L_p_dash_i3; %#ok<NASGU>
  a_xi_pp_dash=x_L_p_dash_i3-x_L_pi3;
  del_pp_dash=(t-t0s)/(t3s-t0s);
  D_del_pp_dash=1/(t3s-t0s); %#ok<NASGU>
  x_ei1=x_ei_t0s+a_xi_pp_dash*(del_pp_dash)^2*(3-2*del_pp_dash);
  D_x_ei1=diff(x_ei1,t);
  DD_x_ei1=diff(D_x_ei1,t);
  
  x_ei=zeros(1,N1);
  D_x_ei=zeros(1,N1);
  DD_x_ei=zeros(1,N1);
  for N=1:N1
      n=(N-1)*h;
      ts1(N)=n;
      
      x_ei(N)=subs(x_ei1,t,ts1(N));
      x_ei(N)=vpa(subs(x_ei(N)));
      D_x_ei(N)=subs(D_x_ei1,t,ts1(N));
      D_x_ei(N)=vpa(subs(D_x_ei(N)));
      DD_x_ei(N)=subs(DD_x_ei1,t,ts1(N));
      DD_x_ei(N)=vpa(subs(DD_x_ei(N)));
  end
  
%------------------------------------------------------------------------
 
 
%x_ei(N)=x_L_p_dash_i3;   %at the end the point have to reach pi3_dash
% D_x_ei(N)=0;   %at the end the point have to reach pi3_dash
% DD_x_ei(N)=0;   %at the end the point have to reach pi3_dash
  
x_ei11=x_ei(1,ceil((ts1_start-ts1_start)/h+1):ceil((ts1_end-ts1_start)/h));
D_x_ei11=D_x_ei(1,ceil((ts1_start-ts1_start)/h+1):ceil((ts1_end-ts1_start)/h));
DD_x_ei11=DD_x_ei(1,ceil((ts1_start-ts1_start)/h+1):ceil((ts1_end-ts1_start)/h));

ts11=ts1;
   end
