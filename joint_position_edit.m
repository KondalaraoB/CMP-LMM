%% function to calculate r_G_pi2_o , r_G_pi1_o  and r_G_si_o values

function [r_G_si_o1a r_G_pi1_oa r_G_pi2_oa r_G0_p0_oa r_G0_pi2_oa r_G0_pi1_oa r_G0_si_o1a D_r_G_pi1_oa D_r_G_pi2_oa DD_r_G_pi1_oa DD_r_G_pi2_oa]=joint_position_edit(theta_i1,beta_i2,beta_i3,N1,r_G_p0_o_t,i)

  m=4;% %can put any value just to call inputs function
    %r_G_p0_o_i=[0 450 150]'*1.0e-3;
[di1,di2,di3,di,Li,Li1,Li2,Li3,Li3p,sai,phi,d,aplha0_i,...
    r_G_p0_o_i,r_L0_si_p0,...
    gama_r,gama_L,etadot0_i,etadot0_f,eta0_i,etaG_i,CC,h,...
    t0,ts0_i1,etadotG_i,etadotG_f,h_Gi3,thetai10]=inputs_edit();

ii=i;
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
    
    
    % define the adress for out put
    r_G_si_o1a=zeros(N1,3);
    r_G_pi1_oa=zeros(N1,3);
    r_G_pi2_oa=zeros(N1,3);
    r_G0_p0_oa=zeros(N1,3);
    r_G0_pi2_oa=zeros(N1,3);
    r_G0_pi1_oa=zeros(N1,3);
    r_G0_si_o1a=zeros(N1,3);
     %######################################################                                          
    t=zeros(N1,1);
    D_r_G_pi1_oa=zeros(N1,3);
    D_r_G_pi2_oa=zeros(N1,3);
    DD_r_G_pi1_oa=zeros(N1,3);
    DD_r_G_pi2_oa=zeros(N1,3);
    
    %########################################################
    % define the adress for transformation matrix storing
    % for A_G_Lipp
    AGLiPP11=zeros(N1,1);
    AGLiPP12=zeros(N1,1);
    AGLiPP13=zeros(N1,1);
    AGLiPP21=zeros(N1,1);
    AGLiPP22=zeros(N1,1);
    AGLiPP23=zeros(N1,1);
    AGLiPP31=zeros(N1,1);
    AGLiPP32=zeros(N1,1);
    AGLiPP33=zeros(N1,1);
    % for A_G_Li1pp
    AGLi1PP11=zeros(N1,1);
    AGLi1PP12=zeros(N1,1);
    AGLi1PP13=zeros(N1,1);
    AGLi1PP21=zeros(N1,1);
    AGLi1PP22=zeros(N1,1);
    AGLi1PP23=zeros(N1,1);
    AGLi1PP31=zeros(N1,1);
    AGLi1PP32=zeros(N1,1);
    AGLi1PP33=zeros(N1,1);
    % for A_G_Li2pp
    AGLi2PP11=zeros(N1,1);
    AGLi2PP12=zeros(N1,1);
    AGLi2PP13=zeros(N1,1);
    AGLi2PP21=zeros(N1,1);
    AGLi2PP22=zeros(N1,1);
    AGLi2PP23=zeros(N1,1);
    AGLi2PP31=zeros(N1,1);
    AGLi2PP32=zeros(N1,1);
    AGLi2PP33=zeros(N1,1);
    %___________________________________________________________
for N=1:N1
    %###############################################################################
    %------------------------------------------------------
    %display(N) %######### to check 
    %------------------------------------------------------
    t(N)=(N-1)*h;
    %a=round(3.44/h);  %####### just to check or find ai, bi, ci value
    t1=t(N);
   %#####################################################################################
%         eta0_ta=eta0_t(N,:);
        %etaG_ta=etaG_t(N,:);
        theta_i1a=theta_i1(N);
        beta_i2a=beta_i2(N);
        beta_i3a=beta_i3(N);
        r_G_p0_o_it=r_G_p0_o_t(N,:);
        %calling transformation matrix A_G_L0 and A_G_G0
        [A_G_L0 A_G_G0]=transfm_edit();
        
        r_G0_p0_oa(N,:)=(A_G_G0'*r_G_p0_o_it')'; %changing to global co-ordinate
       
       %to calculate transfer matrix A_Lo_Li_dashdash
        A_Lo_Ldashdash_i=[cosd(gama-theta_i1a)  sind(gama-theta_i1a)  0
                         -sind(gama-theta_i1a)  cosd(gama-theta_i1a)  0
                          0                    0                    1 ];
               
       A_G_Ldashdash_i= A_G_L0*A_Lo_Ldashdash_i;
       
    %#############################################################18/12/13
    % to assign the value of A_G_Ldashdash_i to the assign address
    AGLiPP11(N,1)=A_G_Ldashdash_i(1,1);
    AGLiPP12(N,1)=A_G_Ldashdash_i(1,2);
    AGLiPP13(N,1)=A_G_Ldashdash_i(1,3);
    AGLiPP21(N,1)=A_G_Ldashdash_i(2,1);
    AGLiPP22(N,1)=A_G_Ldashdash_i(2,2);
    AGLiPP23(N,1)=A_G_Ldashdash_i(2,3);
    AGLiPP31(N,1)=A_G_Ldashdash_i(3,1);
    AGLiPP32(N,1)=A_G_Ldashdash_i(3,2);
    AGLiPP33(N,1)=A_G_Ldashdash_i(3,3);
    %##################################################################
    
 %transfer function from L0 to Ldashdash_i1
 
    A_L0_Ldashdash_i1=[cosd(gama-theta_i1a)*cosd(phi-beta_i2a) sind(gama-theta_i1a) -cosd(gama-theta_i1a)*sind(phi-beta_i2a);
                      -sind(gama-theta_i1a)*cosd(phi-beta_i2a) cosd(gama-theta_i1a) sind(gama-theta_i1a)*sind(phi-beta_i2a);
                       sind(phi-beta_i2a)                        0                cosd(phi-beta_i2a) ];
    
    A_G_Ldashdash_i1= A_G_L0* A_L0_Ldashdash_i1;  
    
    %#####################################################################
    % to assign the value of A_G_Ldashdash_i1 to the assign address
    AGLi1PP11(N,1)=A_G_Ldashdash_i1(1,1);
    AGLi1PP12(N,1)=A_G_Ldashdash_i1(1,2);
    AGLi1PP13(N,1)=A_G_Ldashdash_i1(1,3);
    AGLi1PP21(N,1)=A_G_Ldashdash_i1(2,1);
    AGLi1PP22(N,1)=A_G_Ldashdash_i1(2,2);
    AGLi1PP23(N,1)=A_G_Ldashdash_i1(2,3);
    AGLi1PP31(N,1)=A_G_Ldashdash_i1(3,1);
    AGLi1PP32(N,1)=A_G_Ldashdash_i1(3,2);
    AGLi1PP33(N,1)=A_G_Ldashdash_i1(3,3);
    %#####################################################################

   %transfer function from L0 to Ldashdash_i2
    A_L0_Ldashdash_i2=[cosd(gama-theta_i1a)*cosd(phi-beta_i2a-beta_i3a) sind(gama-theta_i1a) -cosd(gama-theta_i1a)*sind(phi-beta_i2a-beta_i3a);
                      -sind(gama-theta_i1a)*cosd(phi-beta_i2a-beta_i3a) cosd(gama-theta_i1a) sind(gama-theta_i1a)*sind(phi-beta_i2a-beta_i3a);
                       sind(phi-beta_i2a-beta_i3a)                        0                cosd(phi-beta_i2a-beta_i3a) ];
    
    A_G_Ldashdash_i2= A_G_L0* A_L0_Ldashdash_i2; 
    
    %##########################################################################                 
    % to assign the value of A_G_Ldashdash_i2 to the assign address
    AGLi2PP11(N,1)=A_G_Ldashdash_i2(1,1);
    AGLi2PP12(N,1)=A_G_Ldashdash_i2(1,2);
    AGLi2PP13(N,1)=A_G_Ldashdash_i2(1,3);
    AGLi2PP21(N,1)=A_G_Ldashdash_i2(2,1);
    AGLi2PP22(N,1)=A_G_Ldashdash_i2(2,2);
    AGLi2PP23(N,1)=A_G_Ldashdash_i2(2,3);
    AGLi2PP31(N,1)=A_G_Ldashdash_i2(3,1);
    AGLi2PP32(N,1)=A_G_Ldashdash_i2(3,2);
    AGLi2PP33(N,1)=A_G_Ldashdash_i2(3,3);
    
    %#########################################################################
    
    %%calculation of different radius vector of joints w.r.t global movable frame G
  r_G_si_o1=r_G_p0_o_it'+A_G_L0*r_L0_si_p0_ii;   
  r_G_pi1_o=r_G_si_o1+A_G_Ldashdash_i*r_Li_dashdash_pi1_si;
  r_G_pi2_o=r_G_pi1_o+A_G_Ldashdash_i1*r_Li_dashdash_pi2_pi1;
    
  %put the value in adress of r_G_si_o1a,r_G_pi1_oa and r_G_pi2_o for out put
  r_G_si_o1a(N,:)=r_G_si_o1;
  r_G_pi1_oa(N,:)=r_G_pi1_o;
  r_G_pi2_oa(N,:)=r_G_pi2_o;
  
  %#####################################################################                     
  if( N==1)
      D_r_G_pi1_oai=[0.0 0.0 0.0];
      D_r_G_pi2_oai=[0.0 0.0 0.0];      
  else
      D_r_G_pi1_oai=(r_G_pi1_oa(N,:)-r_G_pi1_oa(N-1,:))./h;
      D_r_G_pi2_oai=(r_G_pi2_oa(N,:)-r_G_pi2_oa(N-1,:))./h;

  end
    % putting value in adress
      D_r_G_pi1_oa(N,:)=D_r_G_pi1_oai;
      D_r_G_pi2_oa(N,:)=D_r_G_pi2_oai;
    
  if( N==1)
      DD_r_G_pi1_oai=[0 0 0];
      DD_r_G_pi2_oai=[0 0 0];
      
  else
      DD_r_G_pi1_oai=(D_r_G_pi1_oa(N,:)-D_r_G_pi1_oa(N-1,:))./h;
      DD_r_G_pi2_oai=(D_r_G_pi2_oa(N,:)-D_r_G_pi2_oa(N-1,:))./h;
  end
    % putting value in adress
      
      DD_r_G_pi1_oa(N,:)=DD_r_G_pi1_oai;
      DD_r_G_pi2_oa(N,:)=DD_r_G_pi2_oai;   
    
    
  %###########################################################################
 %changing to global co-ordinate
   r_G0_pi2_oa(N,:)=(A_G_G0'*r_G_pi2_oa(N,:)')';
   r_G0_pi1_oa(N,:)=(A_G_G0'*r_G_pi1_oa(N,:)')';
   r_G0_si_o1a(N,:)=(A_G_G0'*r_G_si_o1a(N,:)')';
end
   

