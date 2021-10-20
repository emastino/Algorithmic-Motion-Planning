%% House Keeping
clc; close all; clear all;

%% Plot PRM, PATH, and obstacles Exercise 1 a) i.
color = [0.125, 0.325, 0.875;
        0.875,0.125,0.325;
        0.325,0.875,0.125;
        0.875,0.325,0.125;
        0.125, 0.875, 0.325;
        0.325,0.125, 0.875];
    
    
% start location (x,y)
start = [ 2,2;
          2,14;
          8,14;
          2,8;
          11,2;
          11,14];
% goal location (x,y)    
goal = [14,14;
        14,2;
        8,2;
        14,8;
        5,14;
        5,2];   
  
    
% Obstacles
ospace= importdata("obstacles.txt");

figure

hold on
for i = 1:size(ospace,1)
   vertices = ospace(i,:);
   pgon = polyshape(vertices(1:2:end),vertices(2:2:end));
   plot(pgon, 'FaceColor', [0.0,0.0,0.0], 'FaceAlpha', 1)
end
axis equal

xlim([0 16]); ylim([0 16]);
xlabel('x axis'); ylabel('y-axis');
grid minor


   
% % Plot Tree
% TREE = importdata("Trees.txt");
% 
% 
% hold on
% for q = 1:size(TREE,1)
%     for j = 1:size(TREE,2)/4
%         index = 4*(j-1);
%         plot([TREE(q,index + 1) TREE(q,index+3)], [TREE(q,index+2) TREE(q,index+4)], '-o', 'MarkerEdgeColor', ...
%                 color(q,:), 'Color',color(q,:),...
%                 'MarkerSize', 2.5)
%     end
% end

% 

path = importdata('Trees_Path.txt');
subpath = zeros(size(path));

fid = fopen('Trees_Path.txt');
line1 = fgetl(fid);
res=line1;
while ischar(line1)
    line1 = fgetl(fid);
    res =char(res,line1)
end
fclose(fid);
for k=1:size(res,1)
  A{k}=str2num(res(k,:))
end

% plot starts and goals
for i = 1:k-1   
    % Start Goal 1
    plot(start(i,1),start(i,2),'o', 'MarkerFaceColor', color(i,:), 'MarkerEdgeColor', color(i,:),...
                'MarkerSize', 5);       
    plot(goal(i,1),goal(i,2),'o', 'MarkerEdgeColor', color(i,:),'MarkerSize', 5); 
end


% make the paths all the same size
% Need to populate nans with the goal values 
max = 0;
index = 1;
for i = 1:k-1
   temp = length(A{i})/2;
   if temp>max
       max = temp;
       index = i;
   end
end

robotPath = zeros(max,2*(k-1));

for i = 1:k-1
    iX = 2*(i-1) +1;
    iY = 2*i;
    
    dataTempLONG = A{i};
    for j = 1:length(A{i})/2
        jX = 2*(j-1) +1;
        jY = 2*j; 
        dataTemp(j,1:2) = dataTempLONG(jX:jY);
    end
    
    dataTemp = flip(dataTemp);
    dataTempLength = length(dataTemp);
    
    robotPath(1:dataTempLength,iX:iY) = dataTemp;
    
    if(dataTempLength < max)
        for j =  dataTempLength+1:max
            robotPath(j,iX:iY) = robotPath(dataTempLength,iX:iY);
        end
    end
    
    clear dataTemp
end


% Plot path 
for i=1:k-1
    iX = 2*(i-1) +1;
    iY = 2*i;
    
    plot(robotPath(:,iX),robotPath(:,iY), '--', 'Color', color(i,:), 'LineWidth', 2)
    
end


% Record and make video of path
video = 0; 
if video ==1
    videoName = 'multiAgentPath.avi';
    v = VideoWriter(videoName);
    v.FrameRate = 10;
    open(v);
end
% plot rumbas
for i = 1:max
    
        
    for j = 1:k-1
        iX = 2*(j-1) +1;
        iY = 2*j;
        h(j) = circle2(robotPath(i,iX),robotPath(i,iY),0.5, color(j,:)); 
    end
    pause(0.25)
    
    if video ==1
        frame = getframe(gcf);
        writeVideo(v,frame);
    end
    if i ~= max
        delete(h)
    end
    
end



if video ==1
    close(v);
%     implay(videoName)
end











% path_h = plot(path(:,1), path(:,2), '--o', 'color', [0.86,0.25,0.1],'LineWidth', 2,...
%     'MarkerFaceColor', [0.86,0.25,0.1], 'MarkerEdgeColor', [0.86,0.25,0.1], ...
%     'MarkerSize', 3.5);
% 
% 
% % Clacultae length of path
% path_length = 0;
% 
% for q = 1:length(path)-1
%     path_length = path_length + sqrt((path(q+1,1)-path(q,1))^2 + (path(q+1,2)-path(q,2))^2);
% end
% fprintf('Path Length: %0.3f \n', path_length)
% 
% titleName = ['Exercise 2 (a) Workspace 2: Path Length ' num2str(path_length)];
% title(titleName);
% 
% legend([start_h, goal_h, path_h], 'Start', 'Goal', 'Path', 'Location', 'NW')
% 
% 
% 
% %% Box Plot Exercise 1 b) ii.
% % benchmark results
% VS = importdata("Tree_Benchmark_validSol.txt")';
% PL = importdata("Tree_Benchmark_pathLen.txt")';
% CT = importdata("Tree_Benchmark_compTime.txt")'/1000000;
% 
% % Valid Solutions Plot
% figure
% % subplot(3,1,1)
% boxplot(VS)
% grid minor
% ylim([0 1.05]);
% ylabel('Successes'); xlabel('Benchmark Parameters');
% title('Valid Solutions = 1, Unsuccessful Solution = 0');
% 
% % Path Length Plot
% figure
% % subplot(3,1,2)
% boxplot(PL)
% grid minor
% ylabel('Path Length'); xlabel('Benchmark Parameters');
% xticklabels({'(200, 1)', '(200, 2)', '(500, 1)', '(500, 2)', '(1000, 1)', '(1000, 2)'})
% title('Path Length');
% 
% % Computation time
% figure
% % subplot(3,1,3)
% boxplot(CT)
% grid minor
% ylabel('Time [s]'); xlabel('Benchmark Parameters');
% xticklabels({'(200, 1)', '(200, 2)', '(500, 1)', '(500, 2)', '(1000, 1)', '(1000, 2)'})
% title('Computation Time');


%% Supplemental Functions

function h = circle2(x,y,r, color)
d = r*2;
px = x-r;
py = y-r;
h = rectangle('Position',[px py d d],'Curvature',[1,1], 'FaceColor', color);
daspect([1,1,1])
end