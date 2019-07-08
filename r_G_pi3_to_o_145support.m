%%
% function to calculate full trajectory,%program to calculate the value of r_G_pi3_o for all position
% %to check
%%
%  clear all; close all;clc;
% for i=1:6
% ii=i;
%%
  function[r_G_pi3_oa,D_r_G_pi3_oa,DD_r_G_pi3_oa,gama,NN1]=r_G_pi3_to_o_edit(i,s0pp,t1minust0,t3minust2,del_t12,theta_c,sw,theta1na,beta2na,beta3na,position_ini)
%%.........................................................................
%inputs
    m=4;                     %can put any value just to call inputs function
[di1,di2,di3,di,Li,Li1,Li2,Li3,Li3p,sai,phi,d,aplha0_i,...
    r_G_p0_o_i,r_L0_si_p0,...
    gama_r,gama_L,etadot0_i,etadot0_f,eta0_i,etaG_i,CC,h,...
    t0,ts0_i1,etadotG_i,etadotG_f,h_Gi3,thetai10]=inputs_edit();


%find the initial positionii=i;
ii=i;

[r_G_pi3_o,r_G_si_o,r_G0_pi3_o,r_G0_si_o,gama,A_L0_Ldashdash_i2,r_Li3_dash_sipi3]=initial_position_edit(theta1na,beta2na,beta3na,position_ini);   % initial position for all leg
%i=input('give the leg number for which trajectory is require\n');

[ts_3i]=time_step_edit(s0pp,t1minust0,t3minust2); 
NN1=ceil(ts_3i/h);
tt=zeros(1,NN1);
x_G_ei=zeros(1,NN1);
y_G_ei=zeros(1,NN1);
z_G_ei=zeros(1,NN1);

D_x_G_ei=zeros(1,NN1);
D_y_G_ei=zeros(1,NN1);
D_z_G_ei=zeros(1,NN1);

DD_x_G_ei=zeros(1,NN1);
DD_y_G_ei=zeros(1,NN1);
DD_z_G_ei=zeros(1,NN1);

r_G_pi3_oa=zeros(NN1,3);
D_r_G_pi3_oa=zeros(NN1,3);
DD_r_G_pi3_oa=zeros(NN1,3);
%%
%--------------------------------------------------------------------------
%   %for leg number 1
  if(ii==1)
%%-------------------------------------------------------------------------
   HCNC=1;
   [t0s,ta,tb,tc,td,te,t3s]=temp_time_step_edit(HCNC,s0pp,t1minust0,t3minust2);   %call the time step
   %for support leg
   ts_start=t0s;%##############################################
   ts_end=tc;%############################################################
   [x_ei11 y_ei11 z_ei11 ts11 D_x_ei11 D_y_ei11 D_z_ei11 DD_x_ei11 DD_y_ei11 DD_z_ei11]=full_support_leg(ii,HCNC,ts_start,ts_end);
     x_G_ei0=x_ei11(1);
     y_G_ei0=y_ei11(1);
     z_G_ei0=z_ei11(1);
     
     D_x_G_ei0=D_x_ei11(1);
     D_y_G_ei0=D_y_ei11(1);
     D_z_G_ei0=D_z_ei11(1);
     
     DD_x_G_ei0=DD_x_ei11(1);
     DD_y_G_ei0=DD_y_ei11(1);
     DD_z_G_ei0=DD_z_ei11(1);
     
     tt0=0;
     N_i=ceil((ts_start)/h);
     N_f=ceil((ts_end)/h);
     N_a=N_f-N_i;
  for N=1:N_a
      NN_a=N_i+N;
      x_G_ei(NN_a)=x_G_ei0+x_ei11(N);
      y_G_ei(NN_a)=y_G_ei0+y_ei11(N);
      z_G_ei(NN_a)=z_G_ei0+z_ei11(N);
      
      D_x_G_ei(NN_a)=D_x_ei11(N);
      D_y_G_ei(NN_a)=D_y_ei11(N);
      D_z_G_ei(NN_a)=D_z_ei11(N);
      
      DD_x_G_ei(NN_a)=DD_x_ei11(N);
      DD_y_G_ei(NN_a)=DD_y_ei11(N);
      DD_z_G_ei(NN_a)=DD_z_ei11(N);
      
      tt(NN_a)=tt0+ts11(N);
  end    


%%
%leg number 2
   elseif(ii==2)
 %%-------------------------------------------------------------------------
   HCNC=1;
   [t0s,ta,tb,tc,td,te,t3s]=temp_time_step_edit(HCNC,s0pp,t1minust0,t3minust2);   %call the time step
   %for swing leg
   ts_start=t0s;%############################################################
   ts_end=tc;%###############################################################
   [x_ei11 y_ei11 z_ei11 ts11 D_x_ei11 D_y_ei11 D_z_ei11 DD_x_ei11 DD_y_ei11 DD_z_ei11]=full_swing_leg(ii,HCNC,ts_start,ts_end,del_t12,sw);
     x_G_ei0=x_ei11(1);
     y_G_ei0=y_ei11(1);
     z_G_ei0=z_ei11(1);
     
     D_x_G_ei0=D_x_ei11(1);
     D_y_G_ei0=D_y_ei11(1);
     D_z_G_ei0=D_z_ei11(1);
     
     DD_x_G_ei0=DD_x_ei11(1);
     DD_y_G_ei0=DD_y_ei11(1);
     DD_z_G_ei0=DD_z_ei11(1);
     
     tt0=0;
     N_i=ceil((ts_start)/h);
     N_f=ceil((ts_end)/h);
     N_a=N_f-N_i;
  for N=1:N_a
      NN_a=N_i+N;
      x_G_ei(NN_a)=x_G_ei0+x_ei11(N);
      y_G_ei(NN_a)=y_G_ei0+y_ei11(N);
      z_G_ei(NN_a)=z_G_ei0+z_ei11(N);
      
      D_x_G_ei(NN_a)=D_x_ei11(N);
      D_y_G_ei(NN_a)=D_y_ei11(N);
      D_z_G_ei(NN_a)=D_z_ei11(N);
      
      DD_x_G_ei(NN_a)=DD_x_ei11(N);
      DD_y_G_ei(NN_a)=DD_y_ei11(N);
      DD_z_G_ei(NN_a)=DD_z_ei11(N);
      
      tt(NN_a)=tt0+ts11(N);
  end    


 %.........................................................................
 %%
%leg number 3
   elseif(ii==3)
 %%-------------------------------------------------------------------------
   HCNC=1;
   [t0s,ta,tb,tc,td,te,t3s]=temp_time_step_edit(HCNC,s0pp,t1minust0,t3minust2);   %call the time step
   %for swing leg
   ts_start=t0s;%############################################################
   ts_end=tc;%###############################################################
   [x_ei11 y_ei11 z_ei11 ts11 D_x_ei11 D_y_ei11 D_z_ei11 DD_x_ei11 DD_y_ei11 DD_z_ei11]=full_swing_leg(ii,HCNC,ts_start,ts_end,del_t12,sw);
     x_G_ei0=x_ei11(1);
     y_G_ei0=y_ei11(1);
     z_G_ei0=z_ei11(1);
     
     D_x_G_ei0=D_x_ei11(1);
     D_y_G_ei0=D_y_ei11(1);
     D_z_G_ei0=D_z_ei11(1);
     
     DD_x_G_ei0=DD_x_ei11(1);
     DD_y_G_ei0=DD_y_ei11(1);
     DD_z_G_ei0=DD_z_ei11(1);
     
     tt0=0;
     N_i=ceil((ts_start)/h);
     N_f=ceil((ts_end)/h);
     N_a=N_f-N_i;
  for N=1:N_a
      NN_a=N_i+N;
      x_G_ei(NN_a)=x_G_ei0+x_ei11(N);
      y_G_ei(NN_a)=y_G_ei0+y_ei11(N);
      z_G_ei(NN_a)=z_G_ei0+z_ei11(N);
      
      D_x_G_ei(NN_a)=D_x_ei11(N);
      D_y_G_ei(NN_a)=D_y_ei11(N);
      D_z_G_ei(NN_a)=D_z_ei11(N);
      
      DD_x_G_ei(NN_a)=DD_x_ei11(N);
      DD_y_G_ei(NN_a)=DD_y_ei11(N);
      DD_z_G_ei(NN_a)=DD_z_ei11(N);
      
      tt(NN_a)=tt0+ts11(N);
  end    


 %.........................................................................
  %.........................................................................
 %%
%leg number 4
   elseif(ii==4)
 %%-------------------------------------------------------------------------
   HCNC=1;
   [t0s,ta,tb,tc,td,te,t3s]=temp_time_step_edit(HCNC,s0pp,t1minust0,t3minust2);   %call the time step
   %for support leg
   ts_start=t0s;%##############################################
   ts_end=tc;%############################################################
   [x_ei11 y_ei11 z_ei11 ts11 D_x_ei11 D_y_ei11 D_z_ei11 DD_x_ei11 DD_y_ei11 DD_z_ei11]=full_support_leg(ii,HCNC,ts_start,ts_end);
     x_G_ei0=x_ei11(1);
     y_G_ei0=y_ei11(1);
     z_G_ei0=z_ei11(1);
     
     D_x_G_ei0=D_x_ei11(1);
     D_y_G_ei0=D_y_ei11(1);
     D_z_G_ei0=D_z_ei11(1);
     
     DD_x_G_ei0=DD_x_ei11(1);
     DD_y_G_ei0=DD_y_ei11(1);
     DD_z_G_ei0=DD_z_ei11(1);
     
     tt0=0;
     N_i=ceil((ts_start)/h);
     N_f=ceil((ts_end)/h);
     N_a=N_f-N_i;
  for N=1:N_a
      NN_a=N_i+N;
      x_G_ei(NN_a)=x_G_ei0+x_ei11(N);
      y_G_ei(NN_a)=y_G_ei0+y_ei11(N);
      z_G_ei(NN_a)=z_G_ei0+z_ei11(N);
      
      D_x_G_ei(NN_a)=D_x_ei11(N);
      D_y_G_ei(NN_a)=D_y_ei11(N);
      D_z_G_ei(NN_a)=D_z_ei11(N);
      
      DD_x_G_ei(NN_a)=DD_x_ei11(N);
      DD_y_G_ei(NN_a)=DD_y_ei11(N);
      DD_z_G_ei(NN_a)=DD_z_ei11(N);
      
      tt(NN_a)=tt0+ts11(N);
  end    


%  %.........................................................................
%%
%for leg number 5
   elseif(ii==5)
   HCNC=1;
   [t0s,ta,tb,tc,td,te,t3s]=temp_time_step_edit(HCNC,s0pp,t1minust0,t3minust2);   %call the time step
   %for support leg
   ts_start=t0s;%##############################################
   ts_end=tc;%############################################################
   [x_ei11 y_ei11 z_ei11 ts11 D_x_ei11 D_y_ei11 D_z_ei11 DD_x_ei11 DD_y_ei11 DD_z_ei11]=full_support_leg(ii,HCNC,ts_start,ts_end);
     x_G_ei0=x_ei11(1);
     y_G_ei0=y_ei11(1);
     z_G_ei0=z_ei11(1);
     
     D_x_G_ei0=D_x_ei11(1);
     D_y_G_ei0=D_y_ei11(1);
     D_z_G_ei0=D_z_ei11(1);
     
     DD_x_G_ei0=DD_x_ei11(1);
     DD_y_G_ei0=DD_y_ei11(1);
     DD_z_G_ei0=DD_z_ei11(1);
     
     tt0=0;
     N_i=ceil((ts_start)/h);
     N_f=ceil((ts_end)/h);
     N_a=N_f-N_i;
  for N=1:N_a
      NN_a=N_i+N;
      x_G_ei(NN_a)=x_G_ei0+x_ei11(N);
      y_G_ei(NN_a)=y_G_ei0+y_ei11(N);
      z_G_ei(NN_a)=z_G_ei0+z_ei11(N);
      
      D_x_G_ei(NN_a)=D_x_ei11(N);
      D_y_G_ei(NN_a)=D_y_ei11(N);
      D_z_G_ei(NN_a)=D_z_ei11(N);
      
      DD_x_G_ei(NN_a)=DD_x_ei11(N);
      DD_y_G_ei(NN_a)=DD_y_ei11(N);
      DD_z_G_ei(NN_a)=DD_z_ei11(N);
      
      tt(NN_a)=tt0+ts11(N);
  end    


     %%
     %for leg number 6
     %.....................................................................
 elseif(ii==6)
 %%-------------------------------------------------------------------------
   HCNC=1;
   [t0s,ta,tb,tc,td,te,t3s]=temp_time_step_edit(HCNC,s0pp,t1minust0,t3minust2);   %call the time step
   %for swing leg
   ts_start=t0s;%############################################################
   ts_end=tc;%###############################################################
   [x_ei11 y_ei11 z_ei11 ts11 D_x_ei11 D_y_ei11 D_z_ei11 DD_x_ei11 DD_y_ei11 DD_z_ei11]=full_swing_leg(ii,HCNC,ts_start,ts_end,del_t12,sw);
     x_G_ei0=x_ei11(1);
     y_G_ei0=y_ei11(1);
     z_G_ei0=z_ei11(1);
     
     D_x_G_ei0=D_x_ei11(1);
     D_y_G_ei0=D_y_ei11(1);
     D_z_G_ei0=D_z_ei11(1);
     
     DD_x_G_ei0=DD_x_ei11(1);
     DD_y_G_ei0=DD_y_ei11(1);
     DD_z_G_ei0=DD_z_ei11(1);
     
     tt0=0;
     N_i=ceil((ts_start)/h);
     N_f=ceil((ts_end)/h);
     N_a=N_f-N_i;
  for N=1:N_a
      NN_a=N_i+N;
      x_G_ei(NN_a)=x_G_ei0+x_ei11(N);
      y_G_ei(NN_a)=y_G_ei0+y_ei11(N);
      z_G_ei(NN_a)=z_G_ei0+z_ei11(N);
      
      D_x_G_ei(NN_a)=D_x_ei11(N);
      D_y_G_ei(NN_a)=D_y_ei11(N);
      D_z_G_ei(NN_a)=D_z_ei11(N);
      
      DD_x_G_ei(NN_a)=DD_x_ei11(N);
      DD_y_G_ei(NN_a)=DD_y_ei11(N);
      DD_z_G_ei(NN_a)=DD_z_ei11(N);
      
      tt(NN_a)=tt0+ts11(N);
  end    

     
 %.........................................................................     
       
  end
 %-------------------------------------------------------------------------
  %%
  %convert to global basis
A_Li3_dashdash_li3=[cosd(theta_c)   sind(theta_c)     0
                     -sind(theta_c)   cosd(theta_c)     0
                          0                 0          1 ];
                      
 A_Li3_li3_dashdash=A_Li3_dashdash_li3';
 r_G_pi3_oaa=zeros(NN1,3);
 D_r_G_pi3_oa1=zeros(NN1,3);
 DD_r_G_pi3_oa1=zeros(NN1,3);
x_G_ei1a=zeros(1,NN1);
y_G_ei1a=zeros(1,NN1);
z_G_ei1a=zeros(1,NN1);
a=[x_G_ei' y_G_ei' z_G_ei'];
for N=1:NN1
     r_G_pi3_oa1=[a(N,1) a(N,2) a(N,3)];
    r_G_pi3_oaa(N,:)=(r_G_pi3_o(ii,:)'+A_Li3_li3_dashdash*r_G_pi3_oa1')';
    r_G_pi3_oaa1=r_G_pi3_oaa(N,:);
    x_G_ei1a(1,N)=r_G_pi3_oaa1(1,1);
    y_G_ei1a(1,N)=r_G_pi3_oaa1(1,2);
    z_G_ei1a(1,N)=r_G_pi3_oaa1(1,3);
    D_r_G_pi3_oa1(N,:)=A_Li3_li3_dashdash*[D_x_G_ei(1,N) D_y_G_ei(1,N) D_z_G_ei(1,N)]';
    DD_r_G_pi3_oa1(N,:)=A_Li3_li3_dashdash*[DD_x_G_ei(1,N) DD_y_G_ei(1,N) DD_z_G_ei(1,N)]';
    
end
%%assign value for function call

r_G_pi3_oa=r_G_pi3_oaa;
D_r_G_pi3_oa=D_r_G_pi3_oa1;
DD_r_G_pi3_oa=DD_r_G_pi3_oa1;


end