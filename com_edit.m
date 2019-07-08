%% program to calculate location of centre of mass 
function [r_G_c0_oa1 r_G_ci1_oa1 r_G_ci2_oa1 r_G_ci3_oa1 m0 mi1a mi2a mi3a]= com_edit(r_G_p0_o_t,N1,gama,theta_i1,beta_i2,beta_i3,i)
%to check
%clear all ; close all; clc;
%%
%call the main input function
    m=4;   % just to call input function
[di1,di2,di3,di,Li,Li1,Li2,Li3,Li3p,sai,phi,d,aplha0_i,...
    r_G_p0_o_i,r_L0_si_p0,...
    gama_r,gama_L,etadot0_i,etadot0_f,eta0_i,etaG_i,CC,h,...
    t0,ts0_i1,etadotG_i,etadotG_f,h_Gi3,thetai10]=inputs_edit();

% 

%% ------------------------------------------------------------------------
% call com inputs 

[mT mL r_L0_cTp0 r_L0_cLp0 m0 r_Li1_dash_ci1_pi1 r_Li2_dash_ci2_pi2 r_Li3_dash_ci3_pi3 mi1a mi2a mi3a]=input_com_edit(i);
%%-------------------------------------------------------------------------

%calculate the combilne com of trunkbody and pay load only
r_L0_c0p0=r_L0_cTp0.*(mT/m0)+r_L0_cLp0.*(mL/m0);



gamaa=gama;
    r_G_c0_oa1=zeros(N1,3);
    r_G_ci1_oa1=zeros(N1,3);
    r_G_ci2_oa1=zeros(N1,3);
    r_G_ci3_oa1=zeros(N1,3);

for N=1:N1

  
         theta_i1a=theta_i1(:,N);
         beta_i2a=beta_i2(:,N);
         beta_i3a=beta_i3(:,N);
         r_L0_si_p0a=r_L0_si_p0(i,:)';
       if (i==2||i==4||i==6)

         
         r_Li_dashdash_pi1_si=  [Li1*cosd(phi)  di1  Li1*sind(phi)]';
         r_Li1_dashdash_pi2_pi1=[Li2 di2 0]';
         r_Li2_dashdash_pi3_pi2=[Li3 0 0]';
         gama=gamaa;
       else
 
          
          r_Li_dashdash_pi1_si=  [-Li1*cosd(phi)  di1  Li1*sind(phi)]';
          r_Li1_dashdash_pi2_pi1=[-Li2 di2 0]';
          r_Li2_dashdash_pi3_pi2=[-Li3 0 0]';
          gama=-gamaa;
       end

       r_G_p0_o_ta=r_G_p0_o_t(N,:)';
       [A_G_L0 A_G_G0 A_G_Li1_dash A_G_Li2_dash A_G_Li3_dash]=transform_com_edit(theta_i1a,beta_i2a,beta_i3a,gama,phi);
       r_G_c0_oa=r_G_p0_o_ta+A_G_L0*r_L0_c0p0;
       r_G_c0_oa1(N,:)=r_G_c0_oa;

        %calculation of r_G_ci1_o
        
        r_G_si_oa=r_G_p0_o_ta+A_G_L0*r_L0_si_p0a;
        r_G_pi1_oa=r_G_si_oa+A_G_Li1_dash*r_Li_dashdash_pi1_si;
        r_G_ci1_oa=r_G_pi1_oa+A_G_Li1_dash*r_Li1_dash_ci1_pi1;
        r_G_ci1_oa1(N,:)=r_G_ci1_oa;

        %calculation of r_G_ci2_o
        r_G_pi2_oa=r_G_pi1_oa+A_G_Li2_dash*r_Li1_dashdash_pi2_pi1;
        r_G_ci2_oa=r_G_pi2_oa+A_G_Li2_dash*r_Li2_dash_ci2_pi2;
        r_G_ci2_oa1(N,:)=r_G_ci2_oa;

        %calculation of r_G_ci3_o
         r_G_pi3_oa=r_G_pi2_oa+A_G_Li3_dash*r_Li2_dashdash_pi3_pi2;
         r_G_ci3_oa=r_G_pi3_oa+A_G_Li3_dash*r_Li3_dash_ci3_pi3;
         r_G_ci3_oa1(N,:)=r_G_ci3_oa;


    
end


