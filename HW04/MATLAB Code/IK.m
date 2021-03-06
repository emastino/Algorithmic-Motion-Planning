%% Inverse Kinematics
% Inverse kinematics of a three revolute manipulator 

%% Hosue keep
% clc; clear all; close all;

%% COnstants
% global positions

function [t1,t2,t3] = IK(dg, a)
% dg = [2; -2; 1]
x = dg(1); y = dg(2);

% bar lengths
% a = [1; 0.5; 1];

% positions in A3 frame
dA3 = [a(3); 0;1];
p = dA3(1); q = dA3(2);
a3 = p;
%% Theta 1 
% Theta 1 can be a large range of things
gamma_o = atan2(y,x);

beta_arg = (a(1)^2 +x^2 +y^2 - (a(2) + a3)^2)/(2*a(1)*sqrt(x^2 + y^2));

theta_2_arg = (a(1)^2 + (a(2) + a3)^2 -x^2 -y^2 )/(2*a(1)*(a(2) + a3));

theta_2_check = 0;

% pick a theta 1
if abs(beta_arg) >1
    if(norm(dg(1:2)) < abs(a(1) - a(2) -a(3)))
%         theta_1 = pi/2
        
        
        if x==0 &&y==0
            theta_1 = 0;
            
            t1 = theta_1;
            x1 = a(1)*cos(theta_1);
            y1 = a(1)*sin(theta_1);

            d = sqrt((x-x1)^2 + (y-y1)^2);

            theta_3_arg = (d^2 - a(2)^2 - a(3)^2)/(2*a(3)*a(2));

            theta_3_temp = acos(theta_3_arg);
            theta_3_plus = atan2(sqrt(1- cos(theta_3_temp)^2), cos(theta_3_temp));
    %         theta_3_minus = atan2(-sqrt(1- cos(theta_3_temp)^2), cos(theta_3_temp)) ;
            theta_3 = theta_3_plus;
            t3 = theta_3;


            beta = acos(( d^2 + a(2)^2-a(3)^2)/(2*d*a(2)));
            alpha = atan2(norm(dg(1:2)), norm([x1,y1]));

            t2 = -(beta+alpha -pi);
        else
            vg = [dg(1:2);0]/norm([dg(1:2);0]);
            zed = cross(vg,[0;0;-1]);
            theta_1 = atan2(zed(2), zed(1));
            t1 = theta_1;
            x1 = a(1)*cos(theta_1);
            y1 = a(1)*sin(theta_1);

            d = sqrt((x-x1)^2 + (y-y1)^2);

            theta_3_arg = (d^2 - a(2)^2 - a(3)^2)/(2*a(3)*a(2));

            theta_3_temp = acos(theta_3_arg);
            theta_3_plus = atan2(sqrt(1- cos(theta_3_temp)^2), cos(theta_3_temp));
    %         theta_3_minus = atan2(-sqrt(1- cos(theta_3_temp)^2), cos(theta_3_temp)) ;
            theta_3 = theta_3_plus;
            t3 = -theta_3;


            beta = acos(( d^2 + a(2)^2-a(3)^2)/(2*d*a(2)));
            alpha = atan2(norm(dg(1:2)), norm([x1,y1]));

            t2 = (beta+alpha -pi);
        end
        
        return;
    end
        
else
    beta = acos(beta_arg);
    theta_2_min = acos(theta_2_arg);
    theta_1_min = gamma_o - beta;
    theta_1_max = beta + gamma_o;
    theta_1 = theta_1_max;
    theta_2 = theta_2_min-pi;
    % negative options
%     theta_1 = -theta_1_max
%     theta_2 = -(theta_2_min-pi)
    
    theta_2_check =1; 
end

% theta_1 = pi/4;
%% Theta 3
x1 = a(1)*cos(theta_1);
y1 = a(1)*sin(theta_1);
d = sqrt((x-x1)^2 + (y-y1)^2);

theta_3_arg = (d^2 - a(2)^2 - a3^2)/(2*a3*a(2));

if theta_2_check ==1
    x2 = x1 + a(2)*cos(theta_1+ theta_2);
    y2 = y1 + a(2)*sin(theta_1+ theta_2);
    
    v2 = [ x2-x1; y2-y1];
    v3 = [dg(1)-x2;dg(2)-y2];
    
    theta_3  = acos(dot(v2,v3)/(norm(v2)*norm(v3)));
    
%     theta_3 = 
else
    if abs(theta_3_arg) > 1 
        theta_3 = pi;
    else
    theta_3_temp = acos(theta_3_arg);
    theta_3_plus = atan2(sqrt(1- cos(theta_3_temp)^2), cos(theta_3_temp));
    theta_3_minus = atan2(-sqrt(1- cos(theta_3_temp)^2), cos(theta_3_temp)) ;
    theta_3 = theta_3_plus;
    end
end
%% Theta 2
fprintf('Theta 2 check = 0')
if theta_2_check ==0
    k = (p^2 +q^2 +2*a(3)*(p*cos(theta_3) - q*sin(theta_3)));
    z = ((p*cos(theta_3))^2 + (q*sin(theta_3))^2 + 2*p*a(3)*cos(theta_3) +a(3)^2 ...
            -2*p*q*cos(theta_3)*sin(theta_3)-2*q*a(3)*sin(theta_3));

    n = 2*a(2)*(p*cos(theta_3) - q*sin(theta_3) +a(3));
    m = 2*a(2)*(p*sin(theta_3) + q*cos(theta_3));

    OMEGA = x^2 + y^ 2 - a(2)^2 - k*(sin(theta_1)*cos(theta_1 ))^2 -z*cos(theta_1)^2 - k*sin(theta_1)^2;

    beta = n; 
    gamma = -m;
    alpha = (k*(cos(theta_1)^2 - sin(theta_1)^2) - z*cos(theta_1)^2);

    fun = @(X) OMEGA - alpha*X^2 - beta*X -gamma*(1-X^2)^(1/2);

    cos_t2 = fzero(fun,0.1);

    theta_2_temp = acos(cos_t2);
    theta_2_plus = atan2(sqrt(1- cos(theta_2_temp)^2), cos(theta_2_temp));
    theta_2_minus = atan2(-sqrt(1- cos(theta_2_temp)^2), cos(theta_2_temp));
    
    theta_2 = theta_2_plus;
end
% plus theta 3
% alpha = x^2 + y^2 - p^2 - q^2 -2*p*a(3)*cos(theta_3_plus) + 2*q*a(3)*sin(theta_3_plus)...
%             - a(3)^2 - a(2)^2;       
% beta = 2*p*a(2)*cos(theta_3_plus) - 2*q*a(2)*sin(theta_3_plus) + 2*a(2)*a(3);
% gamma = -(2*a(2)*p*sin(theta_3_plus) + 2*a(2)*q*cos(theta_3_plus));
% 
% theta_2_arg_plus = (alpha*beta + sqrt((alpha*beta)^2 -(beta^2+gamma^2)*(alpha^2-gamma^2)))/(beta^2 + gamma^2);
% theta_2_arg_minus = (alpha*beta - sqrt((alpha*beta)^2 -(beta^2+gamma^2)*(alpha^2-gamma^2)))/(beta^2 + gamma^2);
% 
% theta_2_plus_temp_t3p = acos(theta_2_arg_plus);   
% theta_2_minus_temp_t3p = acos(theta_2_arg_minus);
% 
% theta_2_plus_plus_t3p = atan2(sqrt(1- cos(theta_2_plus_temp_t3p)^2), cos(theta_2_plus_temp_t3p));
% theta_2_plus_minus_t3p = atan2(-sqrt(1- cos(theta_2_plus_temp_t3p)^2), cos(theta_2_plus_temp_t3p));
% 
% theta_2_minus_plus_t3p = atan2(sqrt(1- cos(theta_2_minus_temp_t3p)^2), cos(theta_2_minus_temp_t3p));
% theta_2_minus_minus_t3p = atan2(-sqrt(1- cos(theta_2_minus_temp_t3p)^2), cos(theta_2_minus_temp_t3p));
% 
% 
% % % minus theta 3
% alpha = x^2 + y^2 - p^2 - q^2 -2*p*a(3)*cos(theta_3_minus) + 2*q*a(3)*sin(theta_3_minus)...
%             - a(3)^2 - a(2)^2;       
% beta = 2*p*a(2)*cos(theta_3_minus) - 2*q*a(2)*sin(theta_3_minus) + 2*a(2)*a(3);
% gamma = -(2*a(2)*p*sin(theta_3_minus) + 2*a(2)*q*cos(theta_3_minus));
% 
% theta_2_arg_plus = (alpha*beta + sqrt((alpha*beta)^2 -(beta^2+gamma^2)*(alpha^2-gamma^2)))/(beta^2 + gamma^2);
% theta_2_arg_minus = (alpha*beta - sqrt((alpha*beta)^2 -(beta^2+gamma^2)*(alpha^2-gamma^2)))/(beta^2 + gamma^2);
% 
% 
% theta_2_plus_temp_t3m = acos(theta_2_arg_plus);
% theta_2_minus_temp_t3m = acos(theta_2_arg_minus);
% 
% theta_2_plus_plus_t3m = atan2(sqrt(1- cos(theta_2_plus_temp_t3m)^2), cos(theta_2_plus_temp_t3m));
% theta_2_plus_minus_t3m = atan2(-sqrt(1- cos(theta_2_plus_temp_t3m)^2), cos(theta_2_plus_temp_t3m));
% 
% theta_2_minus_plus_t3m = atan2(sqrt(1- cos(theta_2_minus_temp_t3m)^2), cos(theta_2_minus_temp_t3m));
% theta_2_minus_minus_t3m = atan2(-sqrt(1- cos(theta_2_minus_temp_t3m)^2), cos(theta_2_minus_temp_t3m));
% 
% theta_2 = theta_2_plus_plus_t3m
% 
% % solutions for with theta 3 plus
% sol_theta3_plus = [ theta_1, theta_1, theta_1, theta_1;
%                     theta_2_plus_plus_t3p,theta_2_plus_minus_t3p,theta_2_minus_plus_t3p,theta_2_minus_minus_t3p;
%                     theta_3_plus,theta_3_plus,theta_3_plus,theta_3_plus]
%                 
% sol_theta3_minus = [ theta_1, theta_1, theta_1, theta_1;
%                     theta_2_plus_plus_t3m,theta_2_plus_minus_t3m,theta_2_minus_plus_t3m,theta_2_minus_minus_t3m;
%                     theta_3_minus,theta_3_minus,theta_3_minus,theta_3_minus]

% end


t1 = real(theta_1);
t2 = real(theta_2);
t3 = real(theta_3);

[t1,t2,t3]
end