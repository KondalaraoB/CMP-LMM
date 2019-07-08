%program to calculate joint angle at different swing condition

 function [theta_i11,beta_i21,beta_i31,theta_dash_i11]=swing_angle_edit(ai,bi,ci,i,t1,gama,s0pp,t1minust0,t3minust2)
%to check
% 
% clear all; clc; close all;
% t1=0.44;
% gama=0;
% ai=0.2060;
% bi=-0.006;
% ci= 0.1318;
% i=1;
% s0pp=(30*10e-4)/6;  
% t1minust0=0.5;
% t3minust2=0.5;

% % HCNC=1;
m=4;%can put any value just to call inputs function
[di1,di2,di3,di,Li,Li1,Li2,Li3,Li3p,sai,phi,d,aplha0_i,...
    r_G_p0_o_i,r_L0_si_p0,...
    gama_r,gama_L,etadot0_i,etadot0_f,eta0_i,etaG_i,CC,h,...
    t0,ts0_i1,etadotG_i,etadotG_f,h_Gi3,thetai10]=inputs_edit() ;

%call the time step value
[ts_3i]=time_step_edit(s0pp,t1minust0,t3minust2);
% eta0_t=eta_01;
t=t1;
    %call eta_0 value
    %[r_G_p0_o_t,eta0_t]=r_G_p0_to_o(m); %#ok<ASGLU>
    
    alpha_0=0;  % along x
    beta_0=0;    % along y
    theta_0=0;   % along z
    c1=cosd(alpha_0);
    s1=sind(alpha_0);
    c2=cosd(beta_0);
    s2=sind(beta_0);
    c3=cosd(theta_0);
    s3=sind(theta_0);
    

  Ki1=-(ai*c2*c3+bi*(c1*s3+s1*s2*c3)+ci*(s1*s3-c1*s2*c3));
     Ki2=-(-ai*c2*s3+bi*(c1*c3-s1*s2*s3)+ci*(s1*c3+c1*s2*s3));
     Ki3=-(ai*s2-bi*s1*c2+ci*c1*c2);
     Ki4=sqrt(Ki1^2+Ki2^2-di^2);
     Ki5=((Ki3-Li1*sind(phi))^2+(Ki4-Li1*cosd(phi))^2-Li2^2-Li3^2)/(2*Li2*Li3);


 % angle for different legs
    
    if(i==2||i==4||i==6)                                                                   %if(or(N==1,N==3,N==5))
     theta_i1=gama_r-(2*atan((Ki1-Ki4)/(di+Ki2)))*180/pi;%####multiply gama_r by pi/180
    theta_dash_i1=180/2+gama_r-theta_i1;%-thetai10; % %#ok<MSNU>
    beta_i2=(phi-2*atan(((Ki3-Li1*sind(phi))+sqrt((Ki3-Li1*sind(phi))^2+(Ki4-Li1*cosd(phi))^2-(Li2+Li3*Ki5)^2))/(Li2+Li3*Ki5+Ki4-Li1*cosd(phi)))*180/pi);
      beta_i3=2*atand(sqrt((1-Ki5)/(1+Ki5)));

      elseif(i==1||i==3||i==5)   
     theta_i1=gama_L-(2*atan((Ki1+Ki4)/(di+Ki2)))*180/pi;%#######initial angle
%      theta_i1=gama_L+theta_i1a;
     theta_dash_i1=-180/2+gama_L-theta_i1;%-thetai10; 
     beta_i2=-(phi-2*atan(((Ki3-Li1*sind(phi))+sqrt((Ki3-Li1*sind(phi))^2+(Ki4-Li1*cosd(phi))^2-(Li2+Li3*Ki5)^2))/(Li2+Li3*Ki5+Ki4-Li1*cosd(phi)))*180/pi);

     beta_i3=-2*atand(sqrt((1-Ki5)/(1+Ki5))); 
    end

theta_i11=theta_i1;
beta_i21=beta_i2;
beta_i31=beta_i3;
theta_dash_i11=theta_dash_i1;