%% House Keeping
clc; close all; clear all;

%% Plot PRM, PATH, and obstacles Exercise 1 a) i.

% Obstacles
ospace= importdata("obstacles_a.txt");

figure

hold on
for i = 1:size(ospace,1)
   vertices = ospace(i,:);
   pgon = polyshape(vertices(1:2:end),vertices(2:2:end));
   plot(pgon, 'FaceColor', [0.0,0.0,0.0])
end
axis equal

xlim([-1 11]); ylim([-3 3]);
xlabel('x axis'); ylabel('y-axis');
grid minor

% Plot PRM
PRM = importdata("PRM_Graph_Ex_1_a_i.txt");
hold on
for j = 1: size(PRM,1)
    
    plot(PRM(j,1), PRM(j,2), 'o', 'MarkerEdgeColor', [0.125, 0.325, 0.875])
    x = 1;
    y = 2;
    while(x <=15)
        if(~isnan(PRM(j,x+2)))
            plot([PRM(j,1) PRM(j,x+2)], [PRM(j,2) PRM(j,y+2)], 'Color', [0.125, 0.325, 0.875])
            x = x+2;
            y = y+2;
        else
            break;
        end
    end
    
end




% plot start and goal 
start = [0,0];
goal = [10,0];
% plot start and goal
start_h = plot (start(1), start(2), 'o','MarkerSize', 10,'MarkerEdgeColor', [0.6,0.05, 0.6], 'MarkerFaceColor', [0.6,0.05, 0.6]); 
goal_h = plot (goal(1), goal(2), 'o', 'MarkerSize',10, 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g'); 



path = importdata("Ex_1_a_i_path.txt");
path_h = plot(path(:,1), path(:,2), '--', 'color', [0.86,0.25,0.1],'LineWidth', 2);


% Clacultae length of path
path_length = 0;

for q = 1:length(path)-1
    path_length = path_length + sqrt((path(q+1,1)-path(q,1))^2 + (path(q+1,2)-path(q,2))^2);
end
fprintf('Path Length: %0.3f \n', path_length)

titleName = ['PRM Exercise 1 (a) i): Path Length ' num2str(path_length)];
title(titleName);

legend([start_h, goal_h, path_h], 'Start', 'Goal', 'Path', 'Location', 'NW')



%% Box Plot Exercise 1 b) ii.
% benchmark results
VS = importdata("PRM_Benchmark_validSol.txt")';
PL = importdata("PRM_Benchmark_pathLen.txt")';
CT = importdata("PRM_Benchmark_compTime.txt")'/1000000;

%%
figure
% subplot(3,1,1)
boxplot(VS)
grid minor
ylim([0 1.05]);
ylabel('Successes'); xlabel('Benchmark Parameters');
xticklabels({'(200; 0:5)', '(200; 1)', '(200; 1:5)', '(200; 2)', '(500; 0:5)', '(500; 1)', '(500; 1:5)', '(500; 2)'})
title('Valid Solutions = 1, Unsuccessful Solution = 0');

figure
% subplot(3,1,2)
boxplot(PL)
grid minor
ylabel('Path Length'); xlabel('Benchmark Parameters');
xticklabels({'(200, 0.5)', '(200, 1)', '(200, 1.5)', '(200, 2)', '(500, 0.5)', '(500, 1)', '(500, 1.5)', '(500, 2)'})
title('Path Length');


figure
% subplot(3,1,3)
boxplot(CT)
grid minor
ylabel('Time [s]'); xlabel('Benchmark Parameters');
xticklabels({'(200, 0.5)', '(200, 1)', '(200, 1.5)', '(200, 2)', '(500, 0.5)', '(500, 1)', '(500, 1.5)', '(500, 2)'})
title('Computation Time');

%% 
weights = eye(3);
for i =1:8
    
    meanVS = mean(VS(:,i));
    meanPL = mean(PL(:,i));
    meanCT = mean(CT(:,i));
    
    if meanPL == 0
        score(i) = 2*meanVS + 0.2/meanCT;
    else
        score(i) = 1*meanVS + 2/meanPL + 0.2/meanCT;
    end
end
score

bestscoreIndex = find(score == max(score))

