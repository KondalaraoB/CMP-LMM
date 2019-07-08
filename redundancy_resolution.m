

function [x_p, y_p, theta_ee] = redundancy_resolution(xq, yq, zq)
%clear;clc;
    global l1 l2 l3 xb yb px py pz zo Mw W x0 d2r


    n = size(xq);
    n = n(1);
    r2d = 180/pi;   %to convert radians to degrees
    d2r = pi/180;
    %Generated end-effector trajectory
    Px=xq;
    Py=yq;
    Pz=zq;

    px=Px(1);py=Py(1);pz=Pz(1);

    for i=1:n
    
        disp(i)

        %W=[10 10 10 10 0]; %weights
        W=[1 1 1 20 20];


        fun= @obj_func;     %objective function
   
        lb = [-180*(pi/180),-90*(pi/180),-90*(pi/180),-5,-5];
        ub = [90*(pi/180),90*(pi/180),90*(pi/180),5,5];

        A = [];
        b = [];
        Aeq = [];
        beq = []; 
        nonlincon = @nlcon;

        x_tp1 =x0;

        [q,fval] = fmincon(fun,x_tp1,A,b,Aeq,beq,lb,ub,nonlincon);
        fun_val(i) = fval;

        x0(1) = q(1);   %updating the robot pose (x_t)
        x0(2) = q(2);
        x0(3) = q(3);

        x0(4) = q(4);
        x0(5) = q(5);
   
       if i<=n-1     %updating to the next point of the trajectory
            px=Px(i+1);py=Py(i+1);pz=Pz(i+1);
       end

       
        t1(i) = q(1)*r2d;  %manipulator joint angles
        t2(i) = q(2)*r2d;
        t3(i) = q(3)*r2d;

        x_p(i) = q(4);     %trunk body trajectory
        y_p(i) = q(5);
       
%for ploting
      Mw_p = [ sin((90*d2r)+x0(1))*((l2*(sin(x0(2))))+(l3*sin(x0(2)+x0(3)))), -cos((90*d2r)+x0(1))*((l2*(cos(x0(2))))+(l3*cos(x0(2)+x0(3)))), -cos((90*d2r)+x0(1))*l3*(cos(x0(2)+x0(3)));
             -cos((90*d2r)+x0(1))*((l2*(sin(x0(2))))+(l3*sin(x0(2)+x0(3)))), -sin((90*d2r)+x0(1))*((l2*(cos(x0(2))))+(l3*cos(x0(2)+x0(3)))), -sin((90*d2r)+x0(1))*l3*(cos(x0(2)+x0(3)));
              0                                             ,  -l2*sin(x0(2))-l3*sin(x0(2)+x0(3))            ,  -l3*sin(x0(2)+x0(3))];
    
      Mw_plot(i) = abs(det(Mw_p))/(l2*l3*(l2+l3)) ;      %manipulability measure
    end
theta_ee = [t1', t2', t3'];    %manipulator joint angles

plot(Mw_plot);

for i = 1: size(x_p,2)-1
    strokeL(i) = sqrt((x_p(i+1)-x_p(i))^2+(y_p(i+1)-y_p(i))^2); 
end
disp(strokeL*1000);
end


%Objective function
function func = obj_func(x)
global l1 l2 l3 xb yb px py pz zo Mw W x0 
func = (W(1)*(x(1)-x0(1))^2+W(2)*(x(2)-x0(2))^2+W(3)*(x(3)-x0(3))^2+W(4)*(x(4)-x0(4))^2+W(5)*(x(5)-x0(5))^2);
end

%Equality and Inequality constraints
function [c,ceq] = nlcon(x)
    global l1 l2 l3 xb yb px py pz zo Mw W x0 d2r
    Mw_matrix = [ sin((90*d2r)+x(1))*(l2*(sin(x(2)))+(l3*sin(x(2)+x(3)))), -cos((90*d2r)+x(1))*(l2*(cos(x(2)))+(l3*cos(x(2)+x(3)))), -cos((90*d2r)+x(1))*l3*(cos(x(2)+x(3)));
                 -cos((90*d2r)+x(1))*(l2*(sin(x(2)))+(l3*sin(x(2)+x(3)))), -sin((90*d2r)+x(1))*(l2*(cos(x(2)))+(l3*cos(x(2)+x(3)))), -sin((90*d2r)+x(1))*l3*(cos(x(2)+x(3)));
                        0                                             ,  -l2*sin(x(2))-l3*sin(x(2)+x(3))            ,  -l3*sin(x(2)+x(3))];

    c =  -abs(det(Mw_matrix))+Mw;     %Inequality constraint

        %Equality constraint
    ceq =  [(x(4)-(cos((90*d2r)+x(1))*(l2*sin(x(2))+l3*sin(x(2)+x(3)))))-px; (x(5)- (sin((90*d2r)+x(1))*(l2*sin(x(2))+l3*sin(x(2)+x(3)))))-py; (0.15+ l1 + l2*cos(x(2)) + l3*cos(x(2)+x(3)))-pz];

end 
