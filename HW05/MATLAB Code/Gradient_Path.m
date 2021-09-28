%% House Keeping
clc; close all; clear all;

%% Exercise 2.a
% Gradient info
% eta = 1; gamma = 1, all Qstar = sqrt(2)

ospace= importdata("obstacles_a.txt");

figure

hold on
for i = 1:size(ospace,1)
   vertices = ospace(i,:);
   pgon = polyshape(vertices(1:2:end),vertices(2:2:end));
   plot(pgon)
end
axis equal
xlim([-2 15]); ylim([-5 5]);
xlabel('x axis'); ylabel('y-axis');
grid minor

% start and goal 
start = [0,0];
goal = [10,0];
% plot start and goal
start_h = plot (start(1), start(2), 'o','MarkerSize', 10,'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r'); 
goal_h = plot (goal(1), goal(2), 'o', 'MarkerSize',10, 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g'); 



path = importdata("gradientPath_a.txt");

path_h = plot(path(:,1), path(:,2), '--k', 'LineWidth', 1.2);
% title('Exercise 2.a : \eta = 1, \xi = 1, all Q* = Square Root of 2')

legend ([start_h, goal_h, path_h], 'Start','Goal', 'Path', 'location', 'nw')

%Note: for crazy path use eta = 50, xi=1 , and Q* = [11, 0.01]


%% Exercise 2.b

% eta = 1; gamma = 3, all Q* = [0.5, 0.01 , 13.0 , 2.0 , 2.0 ]

clear all;

ospace= importdata("obstacles_b.txt");

figure

hold on
for i = 1:size(ospace,1)
   vertices = ospace(i,:);
   pgon = polyshape(vertices(1:2:end),vertices(2:2:end));
   plot(pgon, 'FaceColor', [0.5,0.6, 0.9], 'EdgeColor', [0.5,0.6, 0.9])
end
axis equal
xlim([-1 14]); ylim([-1 14]);
xlabel('x axis'); ylabel('y-axis');
grid minor
% 
% start and goal 
start = [0,0];
goal = [10,10];
% plot start and goal
start_h = plot (start(1), start(2), 'o','MarkerSize', 10,'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
goal_h = plot (goal(1), goal(2), 'o', 'MarkerSize',10, 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g') ;

% path
path = importdata("gradientPath_b.txt");
% plot path
path_h = plot(path(:,1), path(:,2), '--k', 'LineWidth', 1.2);

legend ([start_h, goal_h, path_h], 'Start','Goal', 'Path', 'location', 'nw')


%% Exercise 2.c
% eta = 1; gamma = 3, all Q* = []
clear all;

ospace= importdata("obstacles_c.txt");

figure

hold on
for i = 1:size(ospace,1)
   vertices = ospace(i,:);
   pgon = polyshape(vertices(1:2:end),vertices(2:2:end));
   plot(pgon, 'FaceColor', [0.5,0.6, 0.9], 'EdgeColor', [0.5,0.6, 0.9])
end
axis equal
xlim([-7 36]); ylim([-7 7]);
xlabel('x axis'); ylabel('y-axis');
grid minor
% 
% start and goal 
start = [0,0];
goal = [35,10];
% plot start and goal
start_h = plot (start(1), start(2), 'o','MarkerSize', 10,'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
goal_h = plot (goal(1), goal(2), 'o', 'MarkerSize',10, 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g') ;

% path
path = importdata("gradientPath_c.txt");
% plot path
path_h = plot(path(:,1), path(:,2), '--k', 'LineWidth', 1.2);

legend ([start_h, goal_h, path_h], 'Start','Goal', 'Path', 'location', 'nw')
