%% House Keeping
clc; close all; clear all;

%% Plot PRM, PATH, and obstacles Exercise 1 a) i.
color = [0.125, 0.325, 0.875;
        0.875,0.125,0.325;
        0.325,0.875,0.125;
        0.875,0.325,0.125;
        0.725, 0.875, 0.325;
        0.325,0.125, 0.875];
       
% Obstacles
ospace= importdata("obstacles.txt");
% ospace= importdata("obstacles_fun.txt");

figure('units','normalized','outerposition',[0 0 1 1])

hold on
for i = 1:size(ospace,1)
   vertices = ospace(i,:);
   pgon = polyshape(vertices(1:2:end),vertices(2:2:end));
   plot(pgon, 'FaceColor', [0.0,0.0,0.0], 'FaceAlpha', 1)
end
axis equal
title('Motion Planning with Kinodynamic Constraints: Cart and 3 Trailers');
xlim([-1 41]); ylim([-1 41]);
xlabel('x axis'); ylabel('y-axis');
grid minor

% goal region 
goal = [0,34, 6,34, 6,38, 0,38];
 
pgon_goal = polyshape(goal(1:2:end),goal(2:2:end));
plot(pgon_goal, 'FaceColor', [0.1,0.6,0.2], 'FaceAlpha', 0.3)

% start region 
start = [3,1, 14,1, 14,3, 3,3];
pgon_start = polyshape(start(1:2:end),start(2:2:end));
plot(pgon_start, 'FaceColor', [0.8,0.8,0.1], 'FaceAlpha', 0.3)

%% Plot car
% Record and make video of path
video = 1; 
if video ==1
    videoName = 'tractorTrailer_x.avi';
    v = VideoWriter(videoName);
    v.FrameRate = 30;

    open(v);
end


% % Static
% car = [25.9821, 12.248, 27.9807, 12.1727, 28.0183, 13.172, 26.0198, 13.2472, ;
% 23.563, 11.9894, 25.5532, 12.187, 25.4544, 13.1821, 23.4642, 12.9845, ;
% 21.1896, 11.4308, 23.1394, 11.8759, 22.9169, 12.8508, 20.967, 12.4057, ;
% 18.8899, 10.6046, 20.7761, 11.2695, 20.4437, 12.2126, 18.5575, 11.5477, ;];
% 
% for i = 1:4
%    vertices = car(i,:);
%    pgon = polyshape(vertices(1:2:end),vertices(2:2:end));
%    if(i ==1)
%        plot(pgon, 'FaceColor', [1.0,0.5,0.0], 'FaceAlpha', 1)
%    else
%     plot(pgon, 'FaceColor', [0.8,1.0,0.0], 'FaceAlpha', 1)
%    end
% end

% Dynamic
carPaths  = importdata("cartFile.txt");
carPaths = flip(carPaths);
numberOfSteps = size(carPaths,1)/4;

speed = 2;
for i = 1:speed:numberOfSteps
    
    indexFrom = 1 + 4*(i-1);
    indexTo = 4*i;
    
%     if i > 1
%         delete(p)
%     end
    
   vertices = carPaths(indexFrom:indexTo,:);
   
   for j = 1:4
       pgon = polyshape(vertices(5-j,1:2:end),vertices(5-j,2:2:end));
%        if(j ==4)
%             p(j) = plot(pgon, 'FaceColor', [1.0,0.5,0.0], 'FaceAlpha', 1);
%        else
%             p(j) = plot(pgon, 'FaceColor', [0.0,1.0,0.8], 'FaceAlpha', 1);
%        end

        p(j) = plot(pgon, 'FaceColor', color(5-j,:), 'FaceAlpha', 1);

   end
   if video ==1
        frame = getframe(gcf);
        writeVideo(v,frame);
    end
%    pause(0.01);
end

if video ==1
    close(v);
%     implay(videoName)
end
% 
% path = importdata('Trees_Path.txt');
% subpath = zeros(size(path));
% 
% fid = fopen('Trees_Path.txt');
% line1 = fgetl(fid);
% res=line1;
% while ischar(line1)
%     line1 = fgetl(fid);
%     res =char(res,line1)
% end
% fclose(fid);
% for k=1:size(res,1)
%   A{k}=str2num(res(k,:))
% end
% 
% % plot starts and goals
% for i = 1:k-1   
%     % Start Goal 1
%     start_h = plot(start(i,1),start(i,2),'o', 'MarkerFaceColor', color(i,:), 'MarkerEdgeColor', color(i,:),...
%                 'MarkerSize', 5);       
%     goal_h = plot(goal(i,1),goal(i,2),'o', 'MarkerEdgeColor', color(i,:),'MarkerSize', 20); 
% end
% 
% 
% % make the paths all the same size
% % Need to populate nans with the goal values 
% max = 0;
% index = 1;
% for i = 1:k-1
%    temp = length(A{i})/2;
%    if temp>max
%        max = temp;
%        index = i;
%    end
% end
% 
% robotPath = zeros(max,2*(k-1));
% 
% for i = 1:k-1
%     iX = 2*(i-1) +1;
%     iY = 2*i;
%     
%     dataTempLONG = A{i};
%     for j = 1:length(A{i})/2
%         jX = 2*(j-1) +1;
%         jY = 2*j; 
%         dataTemp(j,1:2) = dataTempLONG(jX:jY);
%     end
%     
%     dataTemp = flip(dataTemp);
%     dataTempLength = length(dataTemp);
%     
%     robotPath(1:dataTempLength,iX:iY) = dataTemp;
%     
%     if(dataTempLength < max)
%         for j =  dataTempLength+1:max
%             robotPath(j,iX:iY) = robotPath(dataTempLength,iX:iY);
%         end
%     end
%     
%     clear dataTemp
% end
% 
% 
% % Plot path 
% for i=1:k-1
%     iX = 2*(i-1) +1;
%     iY = 2*i;
%     
%     plot(robotPath(:,iX),robotPath(:,iY), '--', 'Color', color(i,:), 'LineWidth', 2)
%     
% end
% 
% saveas(gcf, 'Ex_1_b.jpg');
% 
% 
% % Record and make video of path
% video = 0; 
% if video ==1
%     videoName = 'multiAgentPath_Centralized.avi';
%     v = VideoWriter(videoName);
%     v.FrameRate = 10;
%     open(v);
% end
% % plot rumbas
% for i = 1:max
%     
%         
%     for j = 1:k-1
%         iX = 2*(j-1) +1;
%         iY = 2*j;
%         h(j) = circle2(robotPath(i,iX),robotPath(i,iY),0.5, color(j,:)); 
%     end
%     pause(0.25)
%     
%     if video ==1
%         frame = getframe(gcf);
%         writeVideo(v,frame);
%     end
%     if i ~= max
%         delete(h)
%     end
%     
% end
% 
% 
% 
% if video ==1
%     close(v);
% %     implay(videoName)
% end
% 
% 
% 
% 
% % %% Benchmark
% % benchmarks = zeros(100,12);
% % 
% % for i = 1:6
% %     
% %    benchMark_fileName = ['Exercise_1_m_' num2str(i) '.txt'];
% %    
% %    
% %    iX = 2*(i-1) +1;
% %    iY = 2*i; 
% %    benchmarks(:, iX:iY) = importdata(benchMark_fileName);
% % end
% % 
% % 
% % %% Exercise 1.c 
% % figure
% % boxplot(benchmarks(:,1:2:3))
% % title('Tree Size')
% % xlabel('Number of Agents'); ylabel('Number of Nodes');
% % grid minor
% % 
% % saveas(gcf, 'Ex_1_c_TreeSize.jpg');
% % 
% % 
% % figure
% % boxplot(benchmarks(:,2:2:4))
% % title('Computation Time')
% % xlabel('Numer of Agents'); ylabel('Computation Time [\mus]');
% % grid minor
% % 
% % saveas(gcf, 'Ex_1_c_CompTime.jpg');
% % 
% % %% Exercise 1.d
% % figure
% % boxplot(benchmarks(:,5:2:11))
% % title('Tree Size')
% % xlabel('Number of Agents'); ylabel('Number of Nodes');
% % xticklabels({'3', '4', '5', '6'})
% % grid minor
% % saveas(gcf, 'Ex_1_d_TreeSize.jpg');
% % 
% % 
% % figure
% % boxplot(benchmarks(:,6:2:12))
% % title('Computation Time')
% % xlabel('Numer of Agents'); ylabel('Computation Time [\mus]');
% % xticklabels({'3', '4', '5', '6'})
% % grid minor
% % 
% % saveas(gcf, 'Ex_1_d_CompTime.jpg');
% % 
% % 
% % %% Exercise 1.e
% % figure
% % hold on
% % for i = 1:6
% %    iX = 2*(i-1) +1;
% %    plot(i, mean(benchmarks(:, iX)), 'o', 'MarkerSize', 7.5, 'MarkerFaceColor',...
% %        color(1,:), 'MarkerEdgeColor', color(1,:)); 
% % end
% % title('Average Tree Sizes')
% % xlabel('Number of Agents'); ylabel('Number of Nodes');
% % xticks([1 2 3 4 5 6]); xlim([0 7])
% % xticklabels({'1', '2', '3', '4', '5', '6'})
% % grid minor
% % saveas(gcf, 'Ex_1_e_TreeSize.jpg');
% % 
% % figure
% % hold on
% % for i = 1:6
% %    iX = 2*i;
% %    plot(i, mean(benchmarks(:, iX)), 'o', 'MarkerSize', 7.5, 'MarkerFaceColor',...
% %        color(1,:), 'MarkerEdgeColor', color(1,:));  
% % end
% % title('Average Computation Time')
% % xlabel('Number of Agents'); ylabel('Computation Time [\mus]');
% % xticks([1 2 3 4 5 6]); xlim([0 7])
% % xticklabels({'1', '2', '3', '4', '5', '6'})
% % grid minor
% % saveas(gcf, 'Ex_1_e_CompTime.jpg');
%% Supplemental Functions

function h = circle2(x,y,r, color)
d = r*2;
px = x-r;
py = y-r;
h = rectangle('Position',[px py d d],'Curvature',[1,1], 'FaceColor', color);
daspect([1,1,1])
end