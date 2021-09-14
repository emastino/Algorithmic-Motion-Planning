%% Homework 3 supplimentary scripts

%% Exercise 2 - Manipulator

% d = [ 0; 8; 8]; % a values
% 
% theta = [ pi/4; pi/2; -pi/6];
% 
% % a in coordinta frame A1
% a_A1 = [4;0;1];
% 
% % b in coordinate frame A2
% b_A2 = [9;0;1];
% 
% % c in coordinate frame A3
% c_A3 = [9;1;1];
% 
% % Generate transormation matrices
% T = zeros([3,3,3]);
% for j = 1: length(theta) 
%     T(:,:,j) = [cos(theta(j)), -sin(theta(j)), d(j);
%                 sin(theta(j)), cos(theta(j)), 0;
%                 0, 0, 1]; 
% end
% 
% % Global Representations
% a = T(:,:,1)*a_A1;
% 
% b = T(:,:,1)*T(:,:,2)*b_A2;
% 
% c = T(:,:,1)*T(:,:,2)*T(:,:,3)*c_A3;


%% Fun 
clear all;
close all;

% goal location
goal = [8; 0; 1];

% length from jint to joint
d = [ 0; 8; 8]; % a values

% theta vectors
speed = 2*pi/0.1;
n = round(speed);
theta_fin =[1.5708  -1.4455 -1.6891];
theta = [ linspace(pi/2,theta_fin(1),n); linspace(0,theta_fin(2),n); linspace(0,theta_fin(3),n)];


% a in coordinta frame A1
a_ends = [-1 9;0 0;1 1];
a_joint = [0;0;1];
% b in coordinate frame A2
b_ends = [-1 9;0 0;1 1];
b_joint = [0;0;1];
% c in coordinate frame A3
c_ends = [-1 9;0 0;1 1];
c_joint = [0;0;1];


% Generate transormation matrices
T = zeros([3,3,3]);

figure('units','normalized','outerposition',[0.25 0.25 0.5 0.75]); % full screen
ax = gca; % get current axis

ax.XMinorGrid = 1; % grid minor on x
ax.YMinorGrid = 1; % grid minor on x
axis equal % axis equal
ax.XLim = [-30 30]; % set limits for x
ax.YLim = [-30 30]; % set limits for y
hold on

% plot goal
goal= plot(goal(1),goal(2), 'o', 'MarkerSize', 12,'Color', [0.5 0.98 0.1], 'MarkerFaceColor', [0.5 0.98 0.1]);

for q = 1:size(theta,2)
    if q>1
        delete(h)
    end

    for j = 1: length(d)
        T(:,:,j) = [cos(theta(j,q)), -sin(theta(j,q)), d(j);
                    sin(theta(j,q)), cos(theta(j,q)), 0;
                    0, 0, 1]; 
    end

    % Global Representations or arms
    a = T(:,:,1)*a_ends;
    b = T(:,:,1)*T(:,:,2)*b_ends;
    c = T(:,:,1)*T(:,:,2)*T(:,:,3)*c_ends;

    % global representation of joint
    aj = T(:,:,1)*a_joint;
    bj = T(:,:,1)*T(:,:,2)*b_joint;
    cj = T(:,:,1)*T(:,:,2)*T(:,:,3)*c_joint;

    % plot arm
    
    h(1) = plot(a(1,1:2),a(2,1:2), '-o', 'Color', 'b', 'MarkerFaceColor', 'b', 'LineWidth', 1.8);
    h(2) = plot(b(1,1:2),b(2,1:2), '-o', 'Color', 'b', 'MarkerFaceColor', 'b', 'LineWidth', 1.8);
    h(3) = plot(c(1,1:2),c(2,1:2), '-o', 'Color', 'b', 'MarkerFaceColor', 'b', 'LineWidth', 1.8);

    % plot joints
    h(4) = plot(aj(1),aj(2), 'o', 'Color', 'r', 'MarkerFaceColor', 'r', 'MarkerSize',7);
    h(5) = plot(bj(1),bj(2), 'o', 'Color', 'r', 'MarkerFaceColor', 'r', 'MarkerSize',7);
    h(6) = plot(cj(1),cj(2), 'o', 'Color', 'r', 'MarkerFaceColor', 'r', 'MarkerSize',7);
    
    pause(0.01)
    
end

