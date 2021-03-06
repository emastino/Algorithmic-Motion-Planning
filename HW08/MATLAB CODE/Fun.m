%% House Keeping
clc; close all; clear all;

%% Plot PRM, PATH, and obstacles Exercise 1 a) i.
% color = [0.125, 0.325, 0.875;
%         0.875,0.125,0.325;
%         0.325,0.875,0.125;
%         0.875,0.325,0.125;
%         0.725, 0.875, 0.325;
%         0.325,0.125, 0.875];
color = rand(16,3);

for i = 1:16
    color(i,1:3) = [0.25, 0.25, 0.6]
end

% % start location (x,y)
% start = [ 2,2;
%           2,14;
%           8,14;
%           2,8;
%           11,2;
%           11,14];
% % goal location (x,y)    
% goal = [14,14;
%         14,2;
%         8,2;
%         14,8;
%         5,14;
%         5,2];   
  
    
% Obstacles
ospace= importdata("obstacles.txt");

figure

hold on
% for i = 1:size(ospace,1)
%    vertices = ospace(i,:);
%    pgon = polyshape(vertices(1:2:end),vertices(2:2:end));
%    plot(pgon, 'FaceColor', [0.0,0.0,0.0], 'FaceAlpha', 1)
% end

axis equal
title('Decentralized Multi-agent Planning: Hi');
xlim([1 15]); ylim([1 15]);
xlabel('x axis'); ylabel('y-axis');
grid minor


% 

% path = importdata('Trees_Path_Decentralized.txt');
subpath = zeros(size(path));
% 
fid = fopen('Trees_Path_Decentralized.txt');
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

% k = size(path,1)+1;



% % plot starts and goals
% for i = 1:k-1   
%     % Start Goal 1
%     start_h = plot(start(i,1),start(i,2),'o', 'MarkerFaceColor', color(i,:), 'MarkerEdgeColor', color(i,:),...
%                 'MarkerSize', 5);       
%     goal_h = plot(goal(i,1),goal(i,2),'o', 'MarkerEdgeColor', color(i,:),'MarkerSize', 20); 
% end


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
    
%     dataTemp = flip(dataTemp);
    dataTempLength = length(dataTemp);
    
    robotPath(1:dataTempLength,iX:iY) = dataTemp;
    
    if(dataTempLength < max)
        for j =  dataTempLength+1:max
            robotPath(j,iX:iY) = robotPath(dataTempLength,iX:iY);
        end
    end
    
    clear dataTemp
end


% % Plot path 
% for i=1:k-1
%     iX = 2*(i-1) +1;
%     iY = 2*i;
%     
%     plot(robotPath(:,iX),robotPath(:,iY), '--', 'Color', color(i,:), 'LineWidth', 2)
%     
% end

saveas(gcf, 'Ex_2_b.jpg');


% Record and make video of path
video = 1; 
if video ==1
    videoName = 'multiAgentPath_Decentralized.avi';
    v = VideoWriter(videoName);
    v.FrameRate = 10;
    open(v);
end
% plot rumbas
for i = 1:max
    
        
    for j = 1:k-1
        iX = 2*(j-1) +1;
        iY = 2*j;
        h(j) = circle2(robotPath(i,iX),robotPath(i,iY),0.25, color(j,:)); 
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



%% Supplemental Functions

function h = circle2(x,y,r, color)
d = r*2;
px = x-r;
py = y-r;
h = rectangle('Position',[px py d d],'Curvature',[1,1], 'FaceColor', color);
daspect([1,1,1])
end