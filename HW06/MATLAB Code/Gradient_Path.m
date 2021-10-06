%% House Keeping
clc; close all; clear all;


%% Exercise 1 Workspace 2

ospace= importdata("obstacles_b.txt");

figure

hold on
for i = 1:size(ospace,1)
   vertices = ospace(i,:);
   pgon = polyshape(vertices(1:2:end),vertices(2:2:end));
   plot(pgon)
end
axis equal

xlim([-1 14]); ylim([-1 14]);
xlabel('x axis'); ylabel('y-axis');
grid minor

% start and goal 
start = [0,0];
goal = [10,10];
% plot start and goal
start_h = plot (start(1), start(2), 'o','MarkerSize', 10,'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r'); 
goal_h = plot (goal(1), goal(2), 'o', 'MarkerSize',10, 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g'); 



path = importdata("wavefrontPath_b.txt");
path_h = plot(path(:,1), path(:,2), '--k', 'LineWidth', 1.2);
% title('Exercise 2.a : \xi = 1, \eta = 60, d^*_{goal} = 5, and all Q^*_1 = 11 & Q^*_2 = 2.5')
% legend ([start_h, goal_h, path_h], 'Start','Goal', 'Path', 'location', 'nw')

% Clacultae length of path
path_length = 0;

for q = 1:length(path)-1
    path_length = path_length + sqrt((path(q+1,1)-path(q,1))^2 + (path(q+1,2)-path(q,2))^2);
end
fprintf('Path Length: %0.3f \n', path_length)




%% Exercise 1 Workspace 2

ospace= importdata("obstacles_c.txt");

figure

hold on
for i = 1:size(ospace,1)
   vertices = ospace(i,:);
   pgon = polyshape(vertices(1:2:end),vertices(2:2:end));
   plot(pgon)
end
axis equal

xlim([-7 36]); ylim([-7 7]);
xlabel('x axis'); ylabel('y-axis');
grid minor

% start and goal 
start = [0,0];
goal = [35,0];
% plot start and goal
start_h = plot (start(1), start(2), 'o','MarkerSize', 10,'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r'); 
goal_h = plot (goal(1), goal(2), 'o', 'MarkerSize',10, 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g'); 



path = importdata("wavefrontPath_c.txt");
path_h = plot(path(:,1), path(:,2), '--k', 'LineWidth', 1.2);
% title('Exercise 2.a : \xi = 1, \eta = 60, d^*_{goal} = 5, and all Q^*_1 = 11 & Q^*_2 = 2.5')
% legend ([start_h, goal_h, path_h], 'Start','Goal', 'Path', 'location', 'nw')

% Clacultae length of path
path_length = 0;

for q = 1:length(path)-1
    path_length = path_length + sqrt((path(q+1,1)-path(q,1))^2 + (path(q+1,2)-path(q,2))^2);
end
fprintf('Path Length: %0.3f \n', path_length)


