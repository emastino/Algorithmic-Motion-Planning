%% Homework 4


%% House Keep
clear all;
close all;

%% Run

% Specify If doing Forward Kinematics (1) or not (0)
FK = 0;

% goal location
if FK ==0
    goal = [-0.3; 0.3; 1];
end
% bar lengths
ai = [1; 0.5; 1];
% length from joint to joint in previous coordinate frame 
% A1 joint at 0,0, A2 in A1 is at 1, A3 in A2 is 0.5
d = [ 0; ai(1); ai(2)]; % a values


if FK ==0
    if norm(goal(1:2)) > sum(ai)
        error("Arm is too short")
    end
end
% theta vectors
speed = 2*pi/0.1;
n = round(speed);
theta_fin = zeros(1,3);
if FK ==1
    theta = [ linspace(0,pi/6,n); linspace(0,pi/3,n); linspace(0,7*pi/4,n)];
else
    [theta_fin(1),theta_fin(2),theta_fin(3)]  = IK(goal, ai);
    theta = [ linspace(0,theta_fin(1),n); linspace(0,theta_fin(2),n); linspace(0,theta_fin(3),n)];
end


% a in coordinta frame A1
a_ends = [0 ai(1);
            0 0;
            1 1];
a_joint = [0;0;1];
% b in coordinate frame A2
b_ends = [0 ai(2);
            0 0;
            1 1];
b_joint = [0;0;1];
% c in coordinate frame A3
c_ends = [0 ai(3);
            0 0;
            1 1];
c_joint = [0;0;1];


% Generate transormation matrices
T = zeros([3,3,3]);

figure('units','normalized','outerposition',[0.25 0.25 0.5 0.75]); % full screen
ax = gca; % get current axis

ax.XMinorGrid = 1; % grid minor on x
ax.YMinorGrid = 1; % grid minor on x
axis equal % axis equal
ax.XLim = 1.2*[-sum(ai) sum(ai)]; % set limits for x
ax.YLim = 1.2*[-sum(ai) sum(ai)]; % set limits for y
hold on

if FK ==0
    % plot goal
    goalP= plot(goal(1),goal(2), 'o', 'MarkerSize', 12,'Color', [0.5 0.85 0.1], 'MarkerFaceColor', [0.5 0.85 0.1]);
end


%% Plot the reults and animate it 
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
    
    if q ==size(theta,2) && FK == 1
        goalP= plot(c(1,2),c(2,2), 'o', 'MarkerSize', 12,'Color', [0.5 0.85 0.1], 'MarkerFaceColor', [0.5 0.85 0.1]);
    end
    % plot arm
    
    h(1) = plot(a(1,1:2),a(2,1:2), '-o', 'Color', 'b', 'MarkerFaceColor', 'b', 'LineWidth', 1.6);
    h(2) = plot(b(1,1:2),b(2,1:2), '-o', 'Color', 'b', 'MarkerFaceColor', 'b', 'LineWidth', 1.6);
    h(3) = plot(c(1,1:2),c(2,1:2), '-o', 'Color', 'b', 'MarkerFaceColor', 'b', 'LineWidth', 1.6);

    % plot joints
    h(4) = plot(aj(1),aj(2), 'o', 'Color', 'r', 'MarkerFaceColor', 'r', 'MarkerSize',7);
    h(5) = plot(bj(1),bj(2), 'o', 'Color', 'r', 'MarkerFaceColor', 'r', 'MarkerSize',7);
    h(6) = plot(cj(1),cj(2), 'o', 'Color', 'r', 'MarkerFaceColor', 'r', 'MarkerSize',7);
    
    
    
    pause(0.01)
    
end

if FK ==1
    title('Exercise 2.a Forward Kinematics a_1 = 0.5, a_2 = 1, a_3 = 0.5   \theta_1 = \pi/6, \theta_2 = \pi/3, \theta_3 = 7\pi/4')
else
    title('Exercise 2.b Forward Kinematics a_1 = 1, a_2 = 0.5, a_3 = 1   x = 2, y = 0')
end
legend([h(1), h(4), goalP], 'Arm Link', 'Revolute Joint', 'Goal', 'Location', 'NE')
