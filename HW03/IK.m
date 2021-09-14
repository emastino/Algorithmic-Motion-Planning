%% Inverse Kinematics
% Inverse kinematics of a three revolute manipulator 

%% Hosue keep
clc; clear all; close all;

%% COnstants
% global positions
dg = [8; 0; 1]
x = dg(1); y = dg(2);

% positions in A3 frame
dA3 = [9; 0;1];
p = dA3(1); q = dA3(2);

% bar lengths
a = [8; 8; 8];
a3 = p;
%% Theta 1 
% Theta 1 can be a large range of things
gamma_o = atan2(y,x);

beta_arg = (a(1)^2 +x^2 +y^2 - (a(2) + a3)^2)/(2*a(1)*sqrt(x^2 + y^2));

% pick a theta 1
if abs(beta_arg) >1
    if gamma_o== pi/2 || gamma_o == 3*pi/2
        theta_1 = 0;
    elseif x ==0 && y == 0
        theta_1 = pi/4;
    else 
        theta_1 = pi/2;
    end
        
else
    beta = acos(beta_arg);
    theta_1_min = gamma_o - beta;
    theta_1_max = beta +gamma_o;
    theta_1 = theta_1_min;
end

% theta_1 = 0;
%% Theta 3
x1 = a(1)*cos(theta_1);
y1 = a(1)*sin(theta_1);
d = sqrt((x-x1)^2 + (y-y1)^2);

theta_3_arg = (d^2 - a(2)^2 - a3^2)/(2*a3*a(2));

theta_3_temp = acos(theta_3_arg);

theta_3_plus = atan2(sqrt(1- cos(theta_3_temp)^2), cos(theta_3_temp)); 
theta_3_minus = atan2(-sqrt(1- cos(theta_3_temp)^2), cos(theta_3_temp)); 

%% Theta 2

% plus theta 3
alpha = x^2 + y^2 - p^2 - q^2 -2*p*a(3)*cos(theta_3_plus) + 2*q*a(3)*sin(theta_3_plus)...
            - a(3)^2 - a(2)^2;       
beta = 2*p*a(2)*cos(theta_3_plus) - 2*q*a(2)*sin(theta_3_plus) + 2*a(2)*a(3);
gamma = -(2*a(2)*p*sin(theta_3_plus) + 2*a(2)*q*cos(theta_3_plus));

theta_2_arg_plus = (alpha*beta + sqrt((alpha*beta)^2 -(beta^2+gamma^2)*(alpha^2-gamma^2)))/(beta^2 + gamma^2);
theta_2_arg_minus = (alpha*beta - sqrt((alpha*beta)^2 -(beta^2+gamma^2)*(alpha^2-gamma^2)))/(beta^2 + gamma^2);

theta_2_plus_temp_t3p = acos(theta_2_arg_plus);   
theta_2_minus_temp_t3p = acos(theta_2_arg_minus);

theta_2_plus_plus_t3p = atan2(sqrt(1- cos(theta_2_plus_temp_t3p)^2), cos(theta_2_plus_temp_t3p));
theta_2_plus_minus_t3p = atan2(-sqrt(1- cos(theta_2_plus_temp_t3p)^2), cos(theta_2_plus_temp_t3p));

theta_2_minus_plus_t3p = atan2(sqrt(1- cos(theta_2_minus_temp_t3p)^2), cos(theta_2_minus_temp_t3p));
theta_2_minus_minus_t3p = atan2(-sqrt(1- cos(theta_2_minus_temp_t3p)^2), cos(theta_2_minus_temp_t3p));


% % minus theta 3
alpha = x^2 + y^2 - p^2 - q^2 -2*p*a(3)*cos(theta_3_minus) + 2*q*a(3)*sin(theta_3_minus)...
            - a(3)^2 - a(2)^2;       
beta = 2*p*a(2)*cos(theta_3_minus) - 2*q*a(2)*sin(theta_3_minus) + 2*a(2)*a(3);
gamma = -(2*a(2)*p*sin(theta_3_minus) + 2*a(2)*q*cos(theta_3_minus));

theta_2_arg_plus = (alpha*beta + sqrt((alpha*beta)^2 -(beta^2+gamma^2)*(alpha^2-gamma^2)))/(beta^2 + gamma^2);
theta_2_arg_minus = (alpha*beta - sqrt((alpha*beta)^2 -(beta^2+gamma^2)*(alpha^2-gamma^2)))/(beta^2 + gamma^2);


theta_2_plus_temp_t3m = acos(theta_2_arg_plus);
theta_2_minus_temp_t3m = acos(theta_2_arg_minus);

theta_2_plus_plus_t3m = atan2(sqrt(1- cos(theta_2_plus_temp_t3m)^2), cos(theta_2_plus_temp_t3m));
theta_2_plus_minus_t3m = atan2(-sqrt(1- cos(theta_2_plus_temp_t3m)^2), cos(theta_2_plus_temp_t3m));

theta_2_minus_plus_t3m = atan2(sqrt(1- cos(theta_2_minus_temp_t3m)^2), cos(theta_2_minus_temp_t3m));
theta_2_minus_minus_t3m = atan2(-sqrt(1- cos(theta_2_minus_temp_t3m)^2), cos(theta_2_minus_temp_t3m));



% solutions for with theta 3 plus
sol_theta3_plus = [ theta_1, theta_1, theta_1, theta_1;
                    theta_2_plus_plus_t3p,theta_2_plus_minus_t3p,theta_2_minus_plus_t3p,theta_2_minus_minus_t3p;
                    theta_3_plus,theta_3_plus,theta_3_plus,theta_3_plus]
                
sol_theta3_minus = [ theta_1, theta_1, theta_1, theta_1;
                    theta_2_plus_plus_t3m,theta_2_plus_minus_t3m,theta_2_minus_plus_t3m,theta_2_minus_minus_t3m;
                    theta_3_minus,theta_3_minus,theta_3_minus,theta_3_minus]
    