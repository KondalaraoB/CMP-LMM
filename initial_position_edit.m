%program to calculate the initial position of the leg    
%artificial input to check the programme
 function[r_G_pi3_o,r_G_si_o,r_G0_pi3_o,r_G0_si_o,gama,A_L0_Ldashdash_i2,r_Li3_dash_sipi3]=initial_position_edit(theta1n,beta2n,beta3n,position_ini)
    
 
% theta1na = [0 0 0 0 0 0];
% beta2na = [-16.24141519 16.24141519 -16.24141519 16.24141519 -16.24141519 16.24141519];
% beta3na = [-69.45645712 69.45645712 -69.45645712 69.45645712 -69.45645712 69.45645712];
% position_ini = [0;0;0.15];
%       %given dimension of the legs
%         clear all;close all; clc;
% %         ii=3;
       m=4;% %can put any value just to call inputs function
    %r_G_p0_o_i=[0 450 150]'*1.0e-3;
[di1,di2,di3,di,Li,Li1,Li2,Li3,Li3p,sai,phi,d,aplha0_i,...
    r_G_p0_o_i,r_L0_si_p0,...
    gama_r,gama_L,etadot0_i,etadot0_f,eta0_i,etaG_i,CC,h,...
    t0,ts0_i1,etadotG_i,etadotG_f,h_Gi3,thetai10]=inputs_edit(); 

   %change eta value at time t....from initial eta0_i value
%     eta0_t=eta0_i;
%     etaG_t=etaG_i;
   
    %considering the r_G_pi3_o value for all six leg
    r_G_pi3_o=zeros(6,3);
    r_G_si_o=zeros(6,3);
    %-------------------------------------
    %considering the r_G_pi3_o value for all six leg
    r_G0_pi3_o=zeros(6,3); % not needed
    r_G0_si_o=zeros(6,3);   % notr needed
    r_Li3_dash_sipi3=zeros(6,3);
    %choose leg number i.e. ii value
    for ii=1:6
    
    %taking r_L0_si_p0 value according to leg number ii
    r_L0_si_p0_ii=r_L0_si_p0(ii,:)';
    
    %finding local radius vector for either right or left leg   
    if(ii==2||ii==4||ii==6)   %for right leg
        r_Li_dashdash_pi1_si=[Li1*cosd(phi) di1 Li1*sind(phi)]';
        r_Li_dashdash_pi2_pi1=[Li2 di2 0]';
        r_Li_dashdash_pi3_pi2=[Li3 -1*di3 0]';
        
        
    else                    %for rightleft leg -ve s
        r_Li_dashdash_pi1_si=[-1*Li1*cosd(phi) di1 Li1*sind(phi)]';
        r_Li_dashdash_pi2_pi1=[-1*Li2 di2 0]';
        r_Li_dashdash_pi3_pi2=[-1*Li3 -1*di3 0]';
    end;
    
    %choosing of gama 
    if(ii==2||ii==4||ii==6)  
        gama_in=gama_r; %sw since for half cycle one leg move by 2*s0 value
    else
        gama_in=gama_L;
    end;
%--------------------------------------------------------------------
    gama=gama_in;%NB: MAXIMUM VALUE OF GAMMA OCCURS WHEN s0=150mm
    
   if(ii==1)
        theta_i1=theta1n(1);
        beta_i2=beta2n(1);
        beta_i3=beta3n(1); 
   elseif (ii==2) 
        theta_i1=theta1n(2);
        beta_i2=beta2n(2);
        beta_i3=beta3n(2);
   elseif (ii==3)
        theta_i1=theta1n(3);
        beta_i2=beta2n(3);
        beta_i3=beta3n(3);
   elseif (ii==4) 
        theta_i1=theta1n(4);
        beta_i2=beta2n(4);
        beta_i3=beta3n(4);
   elseif(ii==5)
        theta_i1=theta1n(5);
        beta_i2=beta2n(5);
        beta_i3=beta3n(5);
   else
        theta_i1=theta1n(6);
        beta_i2=beta2n(6);
        beta_i3=beta3n(6);
   end

   
                                 % transformation matrix from G to L0
 
%calling transformation matrix A_G_L0 and A_G_G0
    
    [A_G_L0 A_G_G0]=transfm_edit(); %%###########################
     
 %to calculate transfer matrix A_Lo_Li_dashdash
    
    A_Lo_Ldashdash_i=[cosd(gama-theta_i1)  sind(gama-theta_i1)  0
                     -sind(gama-theta_i1)  cosd(gama-theta_i1)  0
                          0                    0                1];
               
    A_G_Ldashdash_i= A_G_L0*A_Lo_Ldashdash_i;
    
 %transfer function from L0 to Ldashdash_i1
 
    A_L0_Ldashdash_i1=[cosd(gama-theta_i1)*cosd(phi-beta_i2) sind(gama-theta_i1) -cosd(gama-theta_i1)*sind(phi-beta_i2);
                      -sind(gama-theta_i1)*cosd(phi-beta_i2) cosd(gama-theta_i1) sind(gama-theta_i1)*sind(phi-beta_i2);
                       sind(phi-beta_i2)                        0                cosd(phi-beta_i2) ];
    
    A_G_Ldashdash_i1= A_G_L0* A_L0_Ldashdash_i1;   

 %transfer function from L0 to Ldashdash_i2
    
    A_L0_Ldashdash_i2=[cosd(gama-theta_i1)*cosd(phi-beta_i2-beta_i3) sind(gama-theta_i1) -cosd(gama-theta_i1)*sind(phi-beta_i2-beta_i3);
                      -sind(gama-theta_i1)*cosd(phi-beta_i2-beta_i3) cosd(gama-theta_i1) sind(gama-theta_i1)*sind(phi-beta_i2-beta_i3);
                       sind(phi-beta_i2-beta_i3)                        0                cosd(phi-beta_i2-beta_i3) ];
    
    A_G_Ldashdash_i2= A_G_L0* A_L0_Ldashdash_i2; 
    
     
  %%calculation of different radius vector of joints w.r.t global movable frame G
  r_G_si_o1=position_ini+A_G_L0*r_L0_si_p0_ii;
  r_G_pi1_o=r_G_si_o1+A_G_Ldashdash_i*r_Li_dashdash_pi1_si;
  r_G_pi2_o=r_G_pi1_o+A_G_Ldashdash_i1*r_Li_dashdash_pi2_pi1;
  r_G_pi3_o1=r_G_pi2_o+A_G_Ldashdash_i2*r_Li_dashdash_pi3_pi2;   
  r_G_pi3_o(ii,:)=r_G_pi3_o1';
  r_G_si_o(ii,:)=r_G_si_o1';
  
  %-----------------------------------------------------------------
  
  %------------------------------------------------------------------
  %to calculate r_Li3_dash_sipi3
  
  r_Li3_dash_sipi3b=[Li1*cosd(beta_i2+beta_i3)+Li2*cosd(beta_i3)+Li3
                                   di1+di2-di3
                      Li1*sind(beta_i2+beta_i3)+Li2*sind(beta_i3) ];
  r_Li3_dash_sipi3(ii,:)=r_Li3_dash_sipi3b';
  %--------------------------------------------------------------------

    r_G0_pi3_o(ii,:)=A_G_G0'*r_G_pi3_o(ii,:)'; % not needed
    r_G0_si_o(ii,:)=A_G_G0'*r_G_si_o(ii,:)';  % notr needed
  
    end
%    display(r_G_pi3_o)   
%   display(r_G_si_o)      