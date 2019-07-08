
%%%%%%%Notes: 
%Coordinated motion planning algorithm for given end-effector trajectory.

%The input to this algorithm is the end effectory trajectory of the
%manipulator.  The output will be the joint angles of manipulator as well
%as legs of the hexapod platform.

%Fixed gait is used (tripod:1/2). stroke (half cycle) length is variying
%between 6 to 60 mm,  which is restricted due to the physical/geometric
%limitation of the body.


digits(8)
clear all;clc;close all;


global l1 l2 l3 xb yb px py pz zo Mw W x0

x0 = [0*(pi/180); -30*(pi/180); -60*(pi/180); 0; 0]; %initial pose: x0 = [theta1, theta2, theta3, x, y]


%All constants
l1=0.0787;    % manipulator link lengths
l2=0.310;
l3=0.220; 
% l_n = sqrt(l1^2+0.310^2+0.220^2);
xb=0;yb=0;zo=0.15;Mw=0.01;
d2r = pi/180;

theta0 = [x0(1), x0(2), x0(3)];
r_G_p0_o_i=[x0(4) x0(5) 0.150]';

%------------------------------------------
%Manipulator End-effector trajectory generation (This is to be modified to
%change the end effectory trajectory which gives [xq, yq, zq].  It is input
%to the algorithm.
%-------------------------------------------

Px_start= r_G_p0_o_i(1)+xb-(cos((90*d2r)+theta0(1))*(l2*sin(theta0(2))+l3*sin(theta0(2)+theta0(3))));
Py_start= r_G_p0_o_i(2)+yb-(sin((90*d2r)+theta0(1))*(l2*sin(theta0(2))+l3*sin(theta0(2)+theta0(3))));
Pz_start = r_G_p0_o_i(3)+ l1 + l2*cos(theta0(2)) + l3*cos(theta0(2)+theta0(3));

%Trajectory 1 (case 1) Curve on right-side (X_Y plane)--------------------------
t_t = [0 0.5 1];
points = [Px_start Py_start; Px_start+0.05 Py_start+0.1; Px_start+0.1 Py_start+0.15 ];
% t = [0 1 2 3 4 5];
% points = [Px_start Py_start; Px_start+0 Py_start+1; Px_start+1 Py_start+1 ; Px_start+0 Py_start+1; Px_start+0 Py_start+2; Px_start+1 Py_start+2];
Px = points(:,1);
Py = points(:,2);
tq = 0:0.2:1;
%tq = 0:0.1:5;
slope0 = 0;
slopeF = 0;
xq = spline(t_t,[slope0; Px; slopeF],tq);xq=xq';
yq = spline(t_t,[slope0; Py; slopeF],tq);yq=yq';
n = size(xq);
zq = Pz_start*ones(n(1),1);
%-------------------------------------------------

%Trajectory 2 (case 2) circle in 3D-----------------------------------
% r_c = sqrt(Px_start^2 + Py_start^2);
% X_c = 0;
% Y_c = 0;
% Angle = 0:10:360;
% 
% Px = X_c + (r_c*cosd(90+Angle));
% Py = Y_c + (r_c*sind(90+Angle));
% n = size(Px);
% n = n(2);
% Pz = Pz_start*ones(n,1); %height of the trajectory considered to be constant
% 
% r_xyz = [Px', Py', Pz];
% Rot_y = [cosd(2) 0 sind(2)
%             0     1    0
%          -sind(2) 0 cosd(2)];
%      
% r_xyzP = zeros(n, 3);
% for i = 1:n
%     r_xyzP(i,:) = Rot_y*r_xyz(i, :)';
% end
% 
% xq = r_xyzP (:,1);
% yq = r_xyzP (:,2);
% zq = r_xyzP (:,3);
%------------------------------------------------------

%Trajectory 3 (case 3) Sine curve on X-Z plane --------------------------
% xq = linspace(Px_start,0.5,10); 
% syms y 
% zq = Pz_start+(0.1*sin(2*2*pi*xq));
% n = size(xq);
% n = n(2);
% yq = Py_start*ones(n,1);
% xq = xq';  zq = zq'; 
%-------------------------------------


%End of manipulator end effectory trajectory generation-------------------
%----------------------------------------------------------------------

ee_traj  = [xq, yq, zq];

%%%%%%%%%%---------------------------------------------------------------
%STEP1: calculation of trunkbody trajectory and manipulator angles through
%optimization
%%%%%%%%%----------------------------------------------------------------
[x_p, y_p, theta_ee] = redundancy_resolution(xq, yq, zq); %not w.r.t time
%End of step1----------------------------------------------------------------
%%

%inputs file
[di1,di2,di3,di,Li,Li1,Li2,Li3,Li3p,sai,phi,d,aplha0_i,...
    r_G_p0_o_i,r_L0_si_p0,...
    gama_r,gama_L,etadot0_i,etadot0_f,eta0_i,etaG_i,CC,h,...
    t0,ts0_i1,etadotG_i,etadotG_f,h_Gi3,thetai10]=inputs_edit();

%initial condition of trunk body leg 
theta1n = [0 0 0 0 0 0];
beta2n = [-16.24141519 16.24141519 -16.24141519 16.24141519 -16.24141519 16.24141519];
beta3n = [-69.45645712 69.45645712 -69.45645712 69.45645712 -69.45645712 69.45645712];

%initial position of the trunk body
position_ini = r_G_p0_o_i;

rePrint = 0;
 
sz_ee = size(x_p,2);

theta_manipulator = [];
angles_manipulator = [];

ee_manipulator = [];
ee_coor_manipulator =[];
Dee_manipulator = [];
Dee_coor_manipulator =[];

ee_manipulator = [];
angles_manipulator = [];
time = 0; counter = 1; time_inc = 0;

%%%%%%%%%%%%%---------------------------------------------------------
%STEP2: Calculation of all joint angles of legs of trunk body (kinematics of trunk body)
%for the obtained trunk body trajectory ([x_p, y_p]) from optimization in STEP1 (i.e.
%redundancy_resolution() function). 
%%%%%%%%%%%%%---------------------------------------------------------
for sz = 1:sz_ee-1
%-------------------------------------------------------------------------- 
%stroke legngth calculation of trunk body, which is distance between two
%points of obtained trunk body trajectory
    strokeL = sqrt((x_p(sz+1)-x_p(sz))^2+(y_p(sz+1)-y_p(sz))^2);  
                                                               
                                                         
%-------------------------------------------------------------------------
% STROKES
%-------------------------------------------------------------------------
    
    s0=strokeL;                  % BODY STROKE s0 for one-half of a cycle
    s0pp=s0/3;                     % each division out of 6 div of a cycle
    sw=2*s0;                     % Swing leg stroke 


%ramp time step (Initial acceleration and final deceleration timings in sec)for trunk body and legs 
    if strokeL < 12*10e-4 
        %--------------------------------------------------------------------------
        %trunk body ramp time step
        %--------------------------------------------------------------------------
        t1minust0=0.3;
        t3minust2=0.3;

        %-------------------------------------------------------------------------
        %ramp time value for each swing step
        del_t12=0.2;
        del_t23=0.2;

        else 

        %trunk body ramp time step
        %--------------------------------------------------------------------------
        t1minust0=0.5;
        t3minust2=0.5;

        %-------------------------------------------------------------------------
        %ramp time value for each swing step
        del_t12=0.4;
        del_t23=0.4;
    end

    %crab angle calculation of the hexapod
    theta_c = atan2d((y_p(sz+1)-y_p(sz)),(x_p(sz+1)-x_p(sz)));

% for defining adress for com cal the N1 value here
[t0,t1,t2,t3]=TB_time_edit(s0pp,t1minust0,t3minust2);
N1=ceil(t3/h);

%define adress for com calculation
prod_mass_radius1=zeros(N1,3);
mass_sum1=0;% initial value for com calculation
sum=0;  % to check the number of leg is six or not

%% --------------------------------------------------------------------------

%initially we need r_G_pi3_si value for that we need r_G_pi3_o and r_G_p0_o_i value
[r_G_p0_o_t,D_r_G_p0_o_t,DD_r_G_p0_o_t,tim,r_G0_p0_o_t]=r_G_p0_to_o_edit(theta_c,s0pp,t1minust0,t3minust2,position_ini); %function calling for r_G_p0_o_i


%----------------------------------------------------------------------
% Manipulator angles : curve fitting with quntic polynomial
theta_manipulator=[];
ee_manipulator=[];
Dee_manipulator=[];

for i_angle = 1:3
   
   %curve fitting for manipulator angles 
    th_i = theta_ee(sz, i_angle);
    th_f = theta_ee(sz+1, i_angle);
    
    Dth_i = 0;
    Dth_f = 0;
    
    DDth_i = 0;
    DDth_f = 0;
    
    [theta_manipulator1]=angle_manipulator(t0, t3, th_i, th_f, Dth_i, Dth_f, DDth_i, DDth_f, h);
    theta_manipulator = [theta_manipulator, theta_manipulator1'];
    
    %curve fitting for manipulator end-effectory trajectory
    ee_i = ee_traj(sz, i_angle);
    ee_f = ee_traj(sz+1, i_angle);
    
    Dee_i = 0;
    Dee_f = 0;
    
    DDee_i = 0;
    DDee_f = 0;
    
    [ee_manipulator1, Dee_manipulator1]=traje_manipulator(t0, t3, ee_i, ee_f, Dee_i, Dee_f, DDee_i, DDee_f, h);
    ee_manipulator = [ee_manipulator, ee_manipulator1'];
    Dee_manipulator = [Dee_manipulator, Dee_manipulator1'];
end

angles_manipulator = [angles_manipulator;theta_manipulator];
ee_coor_manipulator = [ee_coor_manipulator; ee_manipulator];
Dee_coor_manipulator = [Dee_coor_manipulator; Dee_manipulator];

%% --------------------------------------------------------------------------
% r_G_p0_o2=r_G_p0_o_i; %initial value of trunk body position
r_L0_si_p0aa=r_L0_si_p0; 
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% for A_G_L0 
% value
%#########################################################################
a11=zeros(N1,1);
a12=zeros(N1,1);
a13=zeros(N1,1);
a21=zeros(N1,1);
a22=zeros(N1,1);
a23=zeros(N1,1);
a31=zeros(N1,1);
a32=zeros(N1,1);
a33=zeros(N1,1);
%#########################################################################


for i=1:6
    
    display(i)
    ii=i;  % assignnning leg number


% r_G_p0_o1=r_G_p0_o2;
%--------------------------------------------------------------------------
m=4;
[r_G0_pi3_oa,gama,r_G_pi3_oa,D_r_G_pi3_oa,DD_r_G_pi3_oa]=r_G0_pi3_to_o_edit(m,i,s0pp,t1minust0,t3minust2,del_t12,theta_c,sw,theta1n,beta2n,beta3n,position_ini,sz);
% to call the graph in slant surface ..and also m use here is just to call the function 

% __________________________________________________________________________
% adresses
r_L0_si_p01a=r_L0_si_p0(i,:); %for initial condition 
%r_G_p0_o_i=zeros(N1,3);  % for all time step adress
r_G_pi3_o=zeros(N1,3);% adress for all time step for pad location
t=zeros(1,N1);    %adress for time step
theta_i1=zeros(1,N1);  %adress for angle theta
theta_dash_i1=zeros(1,N1);  %adress for angle theta_dash
beta_i2=zeros(1,N1);        %adress for angle beta_i2
beta_i3=zeros(1,N1);        %adress for angle beta_i3
%to check
 ai1=zeros(1,N1);      %adress for ai1
 bi1=zeros(1,N1);      %adress for bi1
 ci1=zeros(1,N1);      %adress for ci1
 %_________________________________________________________________________
 %etaG_t=etaG_i;   % considering plane is constantly sloped
%--------------------------------------------------------------------------
 %input for angle calculation are
 r_G_p0_o_t1=r_G_p0_o_t; %all position vector of trunk body
%  eta0_t1=eta0_t;
 r_G_pi3_oa1=r_G_pi3_oa;
 r_L0_si_p01=r_L0_si_p01a;
% %-------------------------------------------------------------------------- 
% %#######################################-----------------------------------
for N=1:N1
    %------------------------------------------------------
   % display(N) %######### to check 
    %------------------------------------------------------
    t(N)=(N-1)*h;
    %a=round(3.44/h);  %####### just to check or find ai, bi, ci value
    t1=t(N);
    if i==1
       time(counter) = time_inc + h;
       time_inc = time(counter);
       counter = counter+1;
    end
%     r_G_p0_o_i=r_G_p0_o_t1(N,:);
%     eta_01=eta0_t1(N,:)';
    %caling transformation function
    [A_G_L0,A_G_G0]=transfm_edit();%#################################
    
    r_G_pi3_o1=r_G_pi3_oa1(N,:);
    %#############################################################################    
    % put the A_G_L0 value to the corrosponding address
    a11(N,1)=A_G_L0(1,1);
    a12(N,1)=A_G_L0(1,2);
    a13(N,1)=A_G_L0(1,3);
    a21(N,1)=A_G_L0(2,1);
    a22(N,1)=A_G_L0(2,2);
    a23(N,1)=A_G_L0(2,3);
    a31(N,1)=A_G_L0(3,1);
    a32(N,1)=A_G_L0(3,2);
    a33(N,1)=A_G_L0(3,3);
    %------------------------------------------------------
    %------------------------------------------------------
%##################################################################################
    %------------------------------------------------------
    %######### to check
%     if(N>=1&&N<=100)
%     display(r_G_pi3_oa1(N,:))  
%     else
%     end
    %------------------------------------------------------
    r_G_pi3_ob=r_G_pi3_o1';% converting row to columb
    r_G_p0_ob=r_G_p0_o_t(N,:)';
    %------------------------------------------------------
    %######### to check
%     if(N>=1&&N<=100)
%     display(r_G_p0_ob)    %######### to check
%     else
%     end
    %------------------------------------------------------
    r_L0_si_p0b=r_L0_si_p01';  %always constant because it is the value from local point p0
    r_G_pi3_si=(r_G_p0_ob+A_G_L0*r_L0_si_p0b-r_G_pi3_ob)  ; 
    %------------------------------------------------------
    
    %######### to check
    %if(N>=1&&N<=100)
    %display('r_G_pi3_si along y')
%     display(r_G_pi3_si) %to check
   % else
   % end
    %------------------------------------------------------
    ai=r_G_pi3_si(1,1);
    bi=r_G_pi3_si(2,1);
    ci=r_G_pi3_si(3,1);
 

           [theta_i11,beta_i21,beta_i31,theta_dash_i11]=swing_angle_edit(ai,bi,ci,i,t1,gama,s0pp,t1minust0,t3minust2);
            theta_i1(1,N)=theta_i11;
            beta_i2(1,N)=beta_i21;
            beta_i3(1,N)=beta_i31;
            theta_dash_i1(1,N)=theta_dash_i11;  
 
end

%--------------------------------------------------------------------------
%% call the centre of mass position

%required values
%r_G_p0_o_t,eta0_t,etaG_t,N1,gama,theta_i1,beta_i2,beta_i3,i

% since it is iteration for six leg so if all six leg are not run then thevalue of com obtain will not be correct
    sum1=sum+1;
    sum=sum1;

   [r_G_c0_oa1 r_G_ci1_oa1 r_G_ci2_oa1 r_G_ci3_oa1 m0 mi1a mi2a mi3a]= com_edit(r_G_p0_o_t,N1,gama,theta_i1,beta_i2,beta_i3,i);
    mi1a1=mi1a(:,i);
    mi2a1=mi2a(:,i);
    mi3a1=mi3a(:,i);
    prod_mass_radius2=r_G_ci1_oa1.*mi1a1+r_G_ci2_oa1.*mi2a1+r_G_ci3_oa1.*mi3a1;
    prod_mass_radius=prod_mass_radius1+prod_mass_radius2;
    prod_mass_radius1=prod_mass_radius;
    mass_sum2=mi1a1+mi2a1+mi3a1;
    mass_sum=mass_sum1+mass_sum2;
    mass_sum1=mass_sum;

 
%% -------------------------------------------------------------------------
%call joint position 
[r_G_si_o1a r_G_pi1_oa r_G_pi2_oa r_G0_p0_oa r_G0_pi2_oa r_G0_pi1_oa r_G0_si_o1a D_r_G_pi1_oa D_r_G_pi2_oa DD_r_G_pi1_oa DD_r_G_pi2_oa]=joint_position_edit( theta_i1,beta_i2,beta_i3,N1,r_G_p0_o_t,i); %#ok<ASGLU>
%% ------------------------------------------------------------------------


%parameter for trunk body
dTD={'Time','',...
    'x_G_p0_oa','y_G_p0_oa','z_G_p0_oa','',...
    'x_G0_p0_oa','y_G0_p0_oa','z_G0_p0_oa','',...
    'D_x_G_p0_oa','D_y_G_p0_oa','D_z_G_p0_oa','',...
    'DD_x_G_p0_oa','DD_y_G_p0_oa','DD_z_G_p0_oa'};
d = {'Time','',...
    'theta_i1','theta_dash_i1','beta_i2','beta_i3','',...
    'x_G_pi3_oa','y_G_pi3_oa','z_G_pi3_oa','',...
    'D_x_G_pi3_oa','D_y_G_pi3_oa','D_z_G_pi3_oa','',...
    'DD_x_G_pi3_oa','DD_y_G_pi3_oa','DD_z_G_pi3_oa','',...
    'x_G_pi2_oa','y_G_pi2_oa','z_G_pi2_oa','',...
    'D_x_G_pi2_oa','D_y_G_pi2_oa','D_z_G_pi2_oa','',...
    'DD_x_G_pi2_oa','DD_y_G_pi2_oa','DD_z_G_pi2_oa','',...
    'x_G_pi1_oa','y_G_pi1_oa','z_G_pi1_oa','',...
    'D_x_G_pi1_oa','D_y_G_pi1_oa','D_z_G_pi1_oa','',...
    'DD_x_G_pi1_oa','DD_y_G_pi1_oa','DD_z_G_pi1_oa','',...
    'x_G0_pi3_oa','y_G0_pi3_oa','z_G0_pi3_oa','',...
    'x_G_si_oa','y_G_si_oa','z_G_si_oa','',...
    'x_G0_pi2_oa','y_G0_pi2_oa','z_G0_pi2_oa','',...
    'x_G0_pi1_oa','y_G0_pi1_oa','z_G0_pi1_oa','',...
    'x_G0_si_o1a','y_G0_si_o1a','z_G0_si_o1a'};%###### stringvalue LEG NO
%##############################################################################################
%parameter for A_G_L0
AGLOp={'Time','','a11','a12','a13','a21','a22','a23','a31','a32','a33'};
%#############################################################################################
% --------------------------------------------------------------------------
%% --------------------------------------------------------------------------
%values to write in excel sheets
%--------------------------------------------------------------------------
r_G_p0_oa=r_G_p0_o_t;
r_G0_p0_oa=r_G0_p0_o_t;
D_r_G_p0_oa=D_r_G_p0_o_t;
DD_r_G_p0_oa=DD_r_G_p0_o_t;


aa=zeros(1,N1);  % simply to separate the column distinctly
%% --------------------------------------------------------------------------
eTD=[time(1+rePrint:end)',aa',...
    r_G_p0_oa(:,1),r_G_p0_oa(:,2),r_G_p0_oa(:,3),aa',...
    r_G0_p0_oa(:,1),r_G0_p0_oa(:,2),r_G0_p0_oa(:,3),aa',...
    D_r_G_p0_o_t(:,1),D_r_G_p0_o_t(:,2),D_r_G_p0_o_t(:,3),aa',...
    DD_r_G_p0_o_t(:,1),DD_r_G_p0_o_t(:,2),DD_r_G_p0_o_t(:,3),aa'];


e=[time(1+rePrint:end)',aa',...
  theta_i1',theta_dash_i1',beta_i2',beta_i3',aa',...
  r_G_pi3_oa(:,1),r_G_pi3_oa(:,2),r_G_pi3_oa(:,3),aa',...
  D_r_G_pi3_oa(:,1),D_r_G_pi3_oa(:,2),D_r_G_pi3_oa(:,3),aa',...
  DD_r_G_pi3_oa(:,1),DD_r_G_pi3_oa(:,2),DD_r_G_pi3_oa(:,3),aa',...
  r_G_pi2_oa(:,1),r_G_pi2_oa(:,2),r_G_pi2_oa(:,3),aa',...
  D_r_G_pi2_oa(:,1),D_r_G_pi2_oa(:,2),D_r_G_pi2_oa(:,3),aa',...
  DD_r_G_pi2_oa(:,1),DD_r_G_pi2_oa(:,2),DD_r_G_pi2_oa(:,3),aa',...
  r_G_pi1_oa(:,1),r_G_pi1_oa(:,2),r_G_pi1_oa(:,3),aa',...
  D_r_G_pi1_oa(:,1),D_r_G_pi1_oa(:,2),D_r_G_pi1_oa(:,3),aa',...
  DD_r_G_pi1_oa(:,1),DD_r_G_pi1_oa(:,2),DD_r_G_pi1_oa(:,3),aa',...
  r_G0_pi3_oa(:,1),r_G0_pi3_oa(:,2),r_G0_pi3_oa(:,3),aa',...
  r_G_si_o1a(:,1),r_G_si_o1a(:,2),r_G_si_o1a(:,3),aa',...
  r_G0_pi2_oa(:,1),r_G0_pi2_oa(:,2),r_G0_pi2_oa(:,3),aa',...
  r_G0_pi1_oa(:,1),r_G0_pi1_oa(:,2),r_G0_pi1_oa(:,3),aa',...
  r_G0_si_o1a(:,1),r_G0_si_o1a(:,2),r_G0_si_o1a(:,3)];
%#############################################################################################################
% dimension for A_G_L0 value to store in xl sheet
%AGL0d=[t',aa',a11,a12,a13,a21,a22,a23,a31,a32,a33];


xlswrite('data\Param.xls',dTD, 'Trunk_Body', 'A1'); %['Leg',num2str(i)] is to convert the leg number to sheet number
xlswrite('data\Param.xls', eTD, 'Trunk_Body',['A',num2str(2+rePrint)]); %% we can directly write header and value as using %%%headers = {'First', 'Second', 'Third'};...values = {1, 2, 3 ; 4, 5, 'x' ; 7, 8, 9};...xlswrite('myExample.xlsx', [headers; values]);
xlswrite('data\Param.xls', d, ['Leg',num2str(i)], 'A1'); %['Leg',num2str(i)] is to convert the leg number to sheet number
xlswrite('data\Param.xls', e, ['Leg',num2str(i)], ['A',num2str(2+rePrint)]);
%xlswrite('data\Param.xls',AGLOp, 'A_G_L0', 'A1'); %['Leg',num2str(i)] is to convert the leg number to sheet number
%xlswrite('data\Param.xls',AGL0d, 'A_G_L0', ['A',num2str(2+rePrint)]); %% we can directly write header and value as using %%%headers = {'First', 'Second', 'Third'};...values = {1, 2, 3 ; 4, 5, 'x' ; 7, 8, 9};...xlswrite('myExample.xlsx', [headers; values]);

theta1_next(i) = theta_i1(1,N);
beta2_next(i) = beta_i2(1,N);
beta3_next(i) = beta_i3(1,N);


%##########################################################################################################
end
%disp(N);

rePrint = rePrint+N;
position_ini = r_G_p0_oa(N,:)';
%updating initial position
theta1n = theta1_next;
beta2n = beta2_next;
beta3n = beta3_next;
end 

AMEE={'Time','Theta1','Theta2','Theta3'};
angles_manipulator = [time' , angles_manipulator];
xlswrite('data\Param.xls',AMEE, 'manipulator_joint_angles', 'A1'); %['Leg',num2str(i)] is to convert the leg number to sheet number
xlswrite('data\Param.xls', angles_manipulator, 'manipulator_joint_angles','A2');

AMEE_xyz={'Time','x','y','z','Dx','Dy','Dz'};
ee_coor_manipulator = [time' , ee_coor_manipulator, Dee_coor_manipulator];
xlswrite('data\Param.xls',AMEE_xyz, 'manipulator_ee_traj', 'A1'); %['Leg',num2str(i)] is to convert the leg number to sheet number
xlswrite('data\Param.xls', ee_coor_manipulator, 'manipulator_ee_traj','A2');

%..........................................................................
% %centre of mass(com) calculation
% r_G_cm_o=(r_G_c0_oa1.*((m0)/(m0+mass_sum)))+prod_mass_radius.*((1)/(m0+mass_sum));
% 
% r_G0_cm_o=zeros(N1,3);
% for N=1:N1
% %     r_G_p0_o_i=r_G_p0_o_t1(N,:);
% %     eta_01=eta0_t1(N,:);
%     %caling transformation function
%     [A_G_L0,A_G_G0]=transfm_edit();%#################################
%     r_G_cm_oa=r_G_cm_o(N,:)';
%     r_G0_cm_o(N,:)=A_G_G0'*r_G_cm_oa;
% end
% dcom={'Time','',...
%     'x_G_cm_o','y_G_cm_o','z_G_cm_o','',...
%     'x_G0_cm_o','y_G0_cm_o','z_G0_cm_o','',...
%     'D_x_G_cm_o','D_y_G_cm_o','D_z_G_cm_o','',...
%     'DD_x_G_cm_o','DD_y_G_cm_o','DD_z_G_cm_o'};
% ecom=[t',aa',...
%     r_G_cm_o(:,1),r_G_cm_o(:,2),r_G_cm_o(:,3),aa',...
%     r_G0_cm_o(:,1),r_G0_cm_o(:,2),r_G0_cm_o(:,3),aa',...
%     D_r_G_p0_oa(:,1),D_r_G_p0_oa(:,2),D_r_G_p0_oa(:,3),aa',...
%     DD_r_G_p0_oa(:,1),DD_r_G_p0_oa(:,2),DD_r_G_p0_oa(:,3),aa'];
% 
% xlswrite('data\Param.xls',dcom, 'com', 'A1'); %['Leg',num2str(i)] is to convert the leg number to sheet number
% xlswrite('data\Param.xls',ecom, 'com', 'A2');
