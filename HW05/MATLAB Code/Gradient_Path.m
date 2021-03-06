%% House Keeping
clc; close all; clear all;

%% Exercise 2.a
% Gradient info
%  xi = 1; eta = 60; all Qstar = [11, 0.01]; dStar = 5
clc; close all
ospace= importdata("obstacles_a.txt");

figure

hold on
for i = 1:size(ospace,1)
   vertices = ospace(i,:);
   pgon = polyshape(vertices(1:2:end),vertices(2:2:end));
   plot(pgon)
end
axis equal

xlim([-1 14]); ylim([-4 3]);
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
title('Exercise 2.a : \xi = 1, \eta = 60, d^*_{goal} = 5, and all Q^*_1 = 11 & Q^*_2 = 2.5')
% legend ([start_h, goal_h, path_h], 'Start','Goal', 'Path', 'location', 'nw')

% Clacultae length of path
path_length = 0;

for q = 1:length(path)-1
    path_length = path_length + sqrt((path(q+1,1)-path(q,1))^2 + (path(q+1,2)-path(q,2))^2);
end
fprintf('Path Length: %0.3f \n', path_length)


%%
%Note: for crazy path use eta = 50, xi=1 , and Q* = [11, 0.01]

% %% Exercise 2.a VECTOR FIELD
% 
% ospace= importdata("obstacles_a.txt");
% 
% figure
% 
% hold on
% for i = 1:size(ospace,1)
%    vertices = ospace(i,:);
%    pgon = polyshape(vertices(1:2:end),vertices(2:2:end));
%    plot(pgon)
% end
% axis equal
% xlim([-1 14]); ylim([-4 4]);
% xlabel('x axis'); ylabel('y-axis');
% grid minor
% 
% % start and goal 
% start = [0,0];
% goal = [10,0];
% % plot start and goal
% start_h = plot (start(1), start(2), 'o','MarkerSize', 10,'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r'); 
% goal_h = plot (goal(1), goal(2), 'o', 'MarkerSize',10, 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g'); 

%
vectorField = importdata("vectorField.txt");
X = vectorField(:,1);
Y = vectorField(:,2);
U = vectorField(:,3);
V = vectorField(:,4);

xo = 0.145;
xf = 0.89;
yo = 0.111;
yf = 0.927;

for q = 1:length(vectorField)
    if ~isnan(U(q))&& ~isnan(V(q))
            delta  = 0.05;
            n = 0.35;
            x(q,1) = map2range(X(q),min(X),max(X),xo,xf);   
            x(q,2) = map2range(X(q)+ U(q), min(X+ U),max(X+ U),x(q,1)-n,x(q,1)+n);

            y(q,1) = map2range(Y(q),min(Y),max(Y),yo,yf);   
            y(q,2) = map2range(Y(q)+V(q), min(Y+ V),max(Y+ V),y(q,1)-n,y(q,1)+n);
            

            if(x(q,2)>=xo && x(q,2) <= xf)
                if(y(q,2)>=yo && y(q,2) <= yf)
                    a = annotation('arrow',x(q,:),y(q,:));

                    a.LineWidth = 0.75;
                    a.HeadWidth = 7;
                    a.HeadLength = 7;
                end
            end
            
    end
end



%% Exercise 2.b

% eta = 1; gamma = 3, all Q* = [0.5, 0.01 , 13.0 , 2.0 , 2.0 ]

clear all;

ospace= importdata("obstacles_b.txt");

figure

hold on
for i = 1:size(ospace,1)
   vertices = ospace(i,:);
   pgon = polyshape(vertices(1:2:end),vertices(2:2:end));
   plot(pgon);%, 'FaceColor', [0.5,0.6, 0.9], 'EdgeColor', [0.5,0.6, 0.9])
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

title('Workspace 1 Exercise 2.b')
legend ([start_h, goal_h, path_h], 'Start','Goal', 'Path', 'location', 'nw')

% Clacultae length of path
path_length = 0;

for q = 1:length(path)-1
    path_length = path_length + sqrt((path(q+1,1)-path(q,1))^2 + (path(q+1,2)-path(q,2))^2);
end
fprintf('Path Length: %0.3f \n', path_length)


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
goal = [35,0];
% plot start and goal
start_h = plot (start(1), start(2), 'o','MarkerSize', 10,'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
goal_h = plot (goal(1), goal(2), 'o', 'MarkerSize',10, 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g') ;

% path
path = importdata("gradientPath_c.txt");
% plot path
path_h = plot(path(:,1), path(:,2), '--k', 'LineWidth', 1.2);

legend ([start_h, goal_h, path_h], 'Start','Goal', 'Path', 'location', 'nw')

% Clacultae length of path
path_length = 0;

for q = 1:length(path)-1
    path_length = path_length + sqrt((path(q+1,1)-path(q,1))^2 + (path(q+1,2)-path(q,2))^2);
end
fprintf('Path Length: %0.3f \n', path_length)


%% Unique elements
% Source: 
% https://www.mathworks.com/matlabcentral/answers/447927-removing-duplicates-in-an-array
function [uniqueSequence] = DedupSequence (seq)
    % Eliminate sequentially repeated rows
    
    % Create row vector for diff (must transpose if given a column vector)
    if size(seq,1) > 1
        seqCopy = seq(:,1)'; 
    else
        seqCopy = seq;
    end
    uniqueSequence = seq([true, diff(seqCopy)~=0]);
end


function value = map2range(input, fromMin, fromMax, toMin, toMax)
    m = (toMax -toMin)/(fromMax-fromMin);
    b = toMin -m*fromMin;
    value = m*input + b;
    
    if value > toMax
        value = toMax;
    end
    
    if value < toMin
        value = toMin;
    end

end
