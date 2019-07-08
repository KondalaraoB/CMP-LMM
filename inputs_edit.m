%%IMPORTANT NOTES

% CRAB ANGLE +VE, BODY MOVES FROM LEFT TO RT FWD
% CRAB ANGLE -VE,  BODY MOVES FROM LEFT TO RT back

% programme to provide input values 
function [di1,di2,di3,di,Li,Li1,Li2,Li3,Li3p,sai,phi,d,aplha0_i,...
    r_G_p0_o_i,r_L0_si_p0,...
    gama_r,gama_L,etadot0_i,etadot0_f,eta0_i,etaG_i,CC,h,...
    t0,ts0_i1,etadotG_i,etadotG_f,h_Gi3,thetai10]=inputs_edit()

%thetai10 is made zero

%define the leg number 
%i=2;

%-------------------------------------------------------------------------
%FIXED BODY DIMENSIONS
%-------------------------------------------------------------------------
    di1=8*10e-4;                  %transfer distance from joint si and link li1      
    di2=17.75*10e-4;              %transfer distance from link li1 and link li2  
    di3=19.75*10e-4;              %transfer distance from link li2 and link li3
    Li1=83.5*10e-4;               %length from si to pi1
    Li2=119.34*10e-4;             %length from pi1 to pi2
    Li3p=98*10e-4;                %length from pi2 to pi3 prime
    phi=0;                        %angle made by link li1 with z tb plane top surface , always parallel so 0                             
    d=25*10e-4;                   %dia of end effector
    
    %radius vector of r_L0_si_p0
    r_L0_si_p0=zeros(6,3);%#########################################
    r_L0_si_p0(1, :)= [-80.5 191.10 23.5]*10e-4; %*10e-4 to convert into mks system
    r_L0_si_p0(2, :)= [80.5 191.10 23.5]*10e-4;
    r_L0_si_p0(3, :)= [-80.5 -18.70 23.5]*10e-4;
    r_L0_si_p0(4, :)= [80.5 -18.70 23.5]*10e-4;
    r_L0_si_p0(5, :)= [-80.5 -228.5 23.5]*10e-4;
    r_L0_si_p0(6, :)= [80.5 -228.5 23.5]*10e-4;
    %r_L0_si_p0a=r_L0_si_p0;  
  

%-------------------------------------------------------------------------
%INITIAL ELEVATION CONDITION---(FRAME G & G0)
%-------------------------------------------------------------------------

    hs=200e-3;%STAIRCASE HT
    bs=250e-3;%STAIRCASE WD
    etaG_i=[0 0 0]';%[atan(hs/bs) 60/180 0]';     %etaG_i=input('give input vector on rotation of global axix'); 
     %etaG_i is in degrees
    etadotG_i=[0 0 0]';
    etadotG_f=[0 0 0]';
% %etaG IS CONSTANT
% etaG_i=etaG;        
        
%-------------------------------------------------------------------------
%INITIAL DISPLACEMENT CONDITION---TRUNK BODY (FRAME G & L0) at t=0
%-------------------------------------------------------------------------

        eta0_i=[0 0 0]'; %-2 2 0 %eta initial value in degrees
%         eta0_f=[0 0 0]'; %eta final value for first half       
        r_G_p0_o_i=[0 0 0.150]';  %distance of local center p0 from global center o
        aplha0_i=eta0_i(1,1);   %REQUIRED TO FIND 'gama_r' & 'gama_L'
        beta0_i=eta0_i(2,1);
        theta0_i=eta0_i(3,1);

%-------------------------------------------------------------------------
%INITIAL VELOCITY CONDITION----TRUNK BODY (FRAME G & L0) at t=0
%-------------------------------------------------------------------------
%     D_x_i = 0;                           %This is present in time_step_edit & TB_time_edit
%     D_x_f = 15*10e-4;                     % in case of change of value need to do it in above also
%     D_x_G_p0_oi= D_x_i*cosd(theta_c);
%     D_x_G_p0_of= D_x_f*cosd(theta_c); %15mm/s
%     D_y_G_p0_oi= D_x_i*sind(theta_c);
%     D_y_G_p0_of= D_x_f*sind(theta_c);
%     D_z_G_p0_oi=0;
%     D_z_G_p0_of=0; 
%     
%     rdot_G_p0_o_i=[D_x_G_p0_oi D_y_G_p0_oi D_z_G_p0_oi]';        %## h_v0 =formula in notes         % velocity profile height at t0 time
%     rdot_G_p0_o_f=[D_x_G_p0_of D_y_G_p0_of D_z_G_p0_of]';
    
    etadot0_i=[0 0 0]';
    etadot0_f=[0 0 0]'; %.01 .02 0

    
%-------------------------------------------------------------------------
%FORMULATED DIMENSIONS
%-------------------------------------------------------------------------
    
    sai=atand(d/(2*Li3p)); %angle substended by li3 link due to end effector dia    
    Li3=Li3p/cosd(sai);    %length from pi2 to pi3
    di=(di1+di2-di3) ;     %since di is multiply down in eqn by  10e-4 so convert it into mm
%     Li=216.27*10e-4;
    Li=218.5*10e-4;
    gama_r=asin(0*cosd(aplha0_i)/2/Li)*180/pi;%+theta_c;  %rotation forward by 20 degree%###############keep gama_r=0
    gama_L=-asin(0*cosd(aplha0_i)/2/Li)*180/pi;%-theta_c; %keeep gama_L=0   
    thetai10=0;
%-------------------------------------------------------------------------
% inputs for time function
%-------------------------------------------------------------------------
    CC=1; %3 complete cycle
    h=0.02; %step gap in discritisation
    %INCORPORATE RAMP TIME
    

%--------------------------------------------------------------------------
%trunk body ramp time step
%--------------------------------------------------------------------------
% t1minust0=.3;
% t3minust2=.3;
t0=0;

%-------------------------------------------------------------------------
%ramp time value for each swing step
% del_t12=.2;
% del_t23=.2;
ts0_i1=0; % value for swing step time at initial starting 



%-------------------------------------------------------------------------
%for initial_condition_eta

h_Gi3=0;          % height of pi3_dash from global frame
end
      



