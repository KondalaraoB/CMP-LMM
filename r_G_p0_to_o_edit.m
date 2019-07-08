%function to calculate displacement of the trunk budy for step velocity input
function [r_G_p0_o_t,D_r_G_p0_o_t,DD_r_G_p0_o_t,tim,r_G0_p0_o_t]=r_G_p0_to_o_edit(theta_c1,s0pp,t1minust0,t3minust2,position_ini)

digits(8);
%inputs 
 
 
%inputs-r_G_p0_o_i,rdot_G_p0_o_i,rdot_G_p0_o_f,
m=4; %can put any value just to call inputs function
[di1,di2,di3,di,Li,Li1,Li2,Li3,Li3p,sai,phi,d,aplha0_i,...
    r_G_p0_o_i,r_L0_si_p0,...
    gama_r,gama_L,etadot0_i,etadot0_f,eta0_i,etaG_i,CC,h,...
    t0,ts0_i1,etadotG_i,etadotG_f,h_Gi3,thetai10]=inputs_edit();


%---------------------------- ---------------------------------------------
%INITIAL VELOCITY CONDITION----TRUNK BODY (FRAME G & L0) at t=0
%-------------------------------------------------------------------------
    D_x_i = 0;                           %This is present in time_step_edit & TB_time_edit
    D_x_f = 15*10e-4;                     % in case of change of value need to do it in above also
    D_x_G_p0_oi= D_x_i*cosd(theta_c1);
    D_x_G_p0_of= D_x_f*cosd(theta_c1); %15mm/s
    D_y_G_p0_oi= D_x_i*sind(theta_c1);
    D_y_G_p0_of= D_x_f*sind(theta_c1);
    D_z_G_p0_oi=0;
    D_z_G_p0_of=0; 
    
    rdot_G_p0_o_i=[D_x_G_p0_oi D_y_G_p0_oi D_z_G_p0_oi]';        %## h_v0 =formula in notes         % velocity profile height at t0 time
    rdot_G_p0_o_f=[D_x_G_p0_of D_y_G_p0_of D_z_G_p0_of]';
%%
%keep original value
r_G_p0_o1=position_ini;
%%
% find input parameter for step input
a_vel=rdot_G_p0_o_f-rdot_G_p0_o_i;
%a_etadot_0=etadot0_f-etadot0_i;
a_etadot_G=etadotG_f-etadotG_i;

[t0,t1,t2,t3]=TB_time_edit(s0pp,t1minust0,t3minust2); %call time t0,t1,t2,t3 value
D_del_a=1/(t1-t0);
D_del_d=1/(t3-t2);

N1=ceil(t3/h);

%r_G_p0_oy_t0=r_G_p0_o1(2,1);
%%
% define adress
t=zeros(1,N1);
r_G_p0_ot=zeros(N1,3);
D_r_G_p0_ot=zeros(N1,3);
DD_r_G_p0_ot=zeros(N1,3);

D_r_G_p0_o_t=zeros(N1,3);
DD_r_G_p0_o_t=zeros(N1,3);
r_G0_p0_o_t=zeros(N1,3);

%%
% calculation of TB motion
syms tt

for N=1:N1
   t(N)=(N-1)*h;
   time=t(N);
   %---------------------------------------------------------------------
   if(t(N)<=t0||t(N)<=t1)
     del_a=(tt-t0)/(t1-t0);
     D_del_a=diff(del_a,tt);
     r_G_p0_ot11=transpose(r_G_p0_o1+((rdot_G_p0_o_i.*del_a)+(a_vel).*((del_a)^3*(1-del_a/2))).*(1/(D_del_a)));
     D_r_G_p0_ot11=diff(r_G_p0_ot11,tt);
     DD_r_G_p0_ot11=diff(D_r_G_p0_ot11,tt);
      
     r_G_p0_ot(N,:)=vpa(subs(r_G_p0_ot11,tt,t(N)));
     D_r_G_p0_ot(N,:)=vpa(subs(D_r_G_p0_ot11,tt,t(N)));
     DD_r_G_p0_ot(N,:)=vpa(subs(DD_r_G_p0_ot11,tt,t(N)));
       
     r_G_p0_ot1=r_G_p0_ot(N,:)';
   %---------------------------------------------------------------------  
    elseif(t(N)<=t1||t(N)<=t2)
       r_G_p0_ot11=transpose(r_G_p0_ot1+(rdot_G_p0_o_f).*(tt-t1));
       D_r_G_p0_ot11=diff(r_G_p0_ot11,tt);
       DD_r_G_p0_ot11=diff(D_r_G_p0_ot11,tt);
   
       r_G_p0_ot(N,:)=vpa(subs(r_G_p0_ot11,tt,t(N)));
       D_r_G_p0_ot(N,:)=vpa(subs(D_r_G_p0_ot11,tt,t(N)));
       DD_r_G_p0_ot(N,:)=vpa(subs(DD_r_G_p0_ot11,tt,t(N)));
   
       r_G_p0_ot2=r_G_p0_ot(N,:)';
       
       else
       del_d=(tt-t2)/(t3-t2);
       
       r_G_p0_ot11=transpose(r_G_p0_ot2+((rdot_G_p0_o_f.*del_d)-(a_vel.*(del_d^3*(1-del_d/2)))).*(1/(D_del_d)));
       D_r_G_p0_ot11=diff(r_G_p0_ot11,tt);
       DD_r_G_p0_ot11=diff(D_r_G_p0_ot11,tt);
       
       r_G_p0_ot(N,:)=vpa(subs(r_G_p0_ot11,tt,t(N)));
       D_r_G_p0_ot(N,:)=vpa(subs(D_r_G_p0_ot11,tt,t(N)));
       DD_r_G_p0_ot(N,:)=vpa(subs(DD_r_G_p0_ot11,tt,t(N)));
   
   end

   [A_G_L0,A_G_G0]=transfm_edit();
   A_G0_G=A_G_G0';   % using orthogonal properties
   r_G0_p0_o_t(N,:)=A_G0_G*r_G_p0_ot(N,:)';
end
r_G_p0_o_t=r_G_p0_ot;
D_r_G_p0_o_t=D_r_G_p0_ot;
DD_r_G_p0_o_t=DD_r_G_p0_ot;
tim=t;
end
