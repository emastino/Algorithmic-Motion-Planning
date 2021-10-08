%% HW04 MATLAB Visualization Code


%% House Keeping
clc; close all; clear all;

%% Exercise 3.a
% wokspace
ospace= importdata("obstacle_Ex_3_a.txt");
% C-Space representation
cspace_a= importdata("cSpace_Ex_3_a.txt");
% path
path = importdata("wavefrontPath_cSpace_Ex2_a.txt");
theta_1 = path(:,1);
theta_2 = path(:,2);


% C- Space
n = sqrt(size(cspace_a,1));

collisions = flip(reshape(cspace_a(:,3), [n,n]));

% Start
thetaStart = 2*[ 180,360-0];
% Goal
thetaGoal = 2*[ 360,360- 0];

cSpace_obs = imshow(collisions);

hold on;

%     % Point
%     cSpacePath(1) = plot(theta_1(j),theta_2(j), 'o', 'MarkerSize', 10,...
%         'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
% path
cSpacePath(1) = plot(2*theta_1,720-2*theta_2, '--b', ...
    'LineWidth', 2.5);

% start 
cS = plot(thetaStart(1), thetaStart(2), 'o', 'MarkerSize', 10,...
    'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');
% goal
cG = plot(thetaGoal(1), thetaGoal(2), 'o', 'MarkerSize', 10,...
    'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g');


xticks([1/n 0.25 0.5 0.75 1]*n);
xticklabels({'0','90','180','270','360'});
yticks([1/n 0.25 0.5 0.75 1]*n);
yticklabels({'360','270','180','90','0'});

xlabel("\theta_1 [deg]");ylabel("\theta_2 [deg]");

title('Exercise 2.a Workspace vs Configuration Space') ;
grid minor
axis on
legend([cSpacePath cS cG], 'C-Space Path', 'Start', 'Goal', 'Location', 'NE');
hold off
saveas(gcf, 'CSpacePath_Ex_2_a.png')
% Manipulator Images

% Bar info
a = [ 1; 1];
figure
count = 1;
for j = round(linspace(1,length(path),4))
    
    if j > 1
        delete(k);
        delete(h);
        delete(polyPlot);
    end
    
    for i = 1:size(ospace,1)
       vertices = ospace(i,:);
       pgon = polyshape(vertices(1:2:end),vertices(2:2:end));
       polyPlot = plot(pgon);
    end
    
    
    a1 = [ a(1)*cosd(theta_1(j)), a(1)*sind(theta_1(j))];
    a2 = [ a1(1) + a(2)*cosd(theta_1(j)+theta_2(j)), a1(2) + a(2)*sind(theta_1(j)+theta_2(j))];
    
    hold on
    % start 
    oS = plot(-2, 0, 'o', 'MarkerSize', 10,...
        'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');
    % goal
    oG = plot(2, 0, 'o', 'MarkerSize', 10,...
        'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g');
    
    h = plot([0 a1(1)], [0 a1(2)], '-o', 'LineWidth', 2.5, 'Color', 'b',...
        'MarkerFaceColor',[0.8, 0.2, 0.2], 'MarkerEdgeColor',[0.8, 0.2, 0.2] );
    k = plot([a1(1) a2(1)], [a1(2) a2(2)], '-o', 'LineWidth', 2.5, 'Color', 'b',...
        'MarkerFaceColor',[0.8, 0.2, 0.2], 'MarkerEdgeColor',[0.8, 0.2, 0.2] );
    
    axis equal
    xlim([-2.5 2.5]); ylim([-2.5 2.5]);
    xlabel('x axis'); ylabel('y-axis');
    grid minor
    titleName = ['Manipulator Position: \theta_1 = ' num2str(theta_1(j)) ', \theta_2 = ' num2str(theta_2(j)) ' Snapshot #' num2str(count)];
    title(titleName);
    
    legend([h, oS, oG], 'Manipulator', 'Start', 'Goal', 'Location', 'NE')
    
    hold off
    
    figureName = ['Manip_Arm_Ex_2_a_' num2str(count) '.png'];
    saveas(gcf, figureName)
    
    
    pause(0.1);
    count = count + 1;
end  


%% Exercise 3.b
clear all
% wokspace
ospace= importdata("obstacle_Ex_3_b.txt");
% C-Space representation
cspace_a= importdata("cSpace_Ex_3_b.txt");
% path
path = importdata("wavefrontPath_cSpace_Ex2_b.txt");
theta_1 = path(:,1);
theta_2 = path(:,2);

% C-Space
n = sqrt(size(cspace_a,1));

collisions = flip(reshape(cspace_a(:,3), [n,n]));

% Start
thetaStart = 2*[ 180,360-0];
% Goal
thetaGoal = 2*[ 360,360- 0];

cSpace_obs = imshow(collisions);

hold on;

%     % Point
%     cSpacePath(1) = plot(theta_1(j),theta_2(j), 'o', 'MarkerSize', 10,...
%         'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
% path
cSpacePath(1) = plot(2*theta_1,720-2*theta_2, '--b', ...
    'LineWidth', 2.5);

% start 
cS = plot(thetaStart(1), thetaStart(2), 'o', 'MarkerSize', 10,...
    'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');
% goal
cG = plot(thetaGoal(1), thetaGoal(2), 'o', 'MarkerSize', 10,...
    'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g');


xticks([1/n 0.25 0.5 0.75 1]*n);
xticklabels({'0','90','180','270','360'});
yticks([1/n 0.25 0.5 0.75 1]*n);
yticklabels({'360','270','180','90','0'});

xlabel("\theta_1 [deg]");ylabel("\theta_2 [deg]");

title('Exercise 2.b Workspace vs Configuration Space') ;
grid minor
axis on
legend([cSpacePath cS cG], 'C-Space Path', 'Start', 'Goal', 'Location', 'NE');
hold off
saveas(gcf, 'CSpacePath_Ex_2_b.png')
% Manipulator Images

% Bar info
a = [ 1; 1];
figure
count = 1;
for j = round(linspace(1,length(path),6))
    
    if j > 1
        delete(h);
        delete(k);
        delete(polyPlot)
    end
    
    
    for i = 1:size(ospace,1)
       vertices = ospace(i,:);
       pgon(i) = polyshape(vertices(1:2:end),vertices(2:2:end));    
    end
    
    polyPlot = plot(pgon,'FaceColor', [0.2,0.3,0.76]);
    
    a1 = [ a(1)*cosd(theta_1(j)), a(1)*sind(theta_1(j))];
    a2 = [ a1(1) + a(2)*cosd(theta_1(j)+theta_2(j)), a1(2) + a(2)*sind(theta_1(j)+theta_2(j))];
    
    hold on
    % start 
    oS = plot(-2, 0, 'o', 'MarkerSize', 10,...
        'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');
    % goal
    oG = plot(2, 0, 'o', 'MarkerSize', 10,...
        'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g');
    
    h = plot([0 a1(1)], [0 a1(2)], '-o', 'LineWidth', 2.5, 'Color', 'b',...
        'MarkerFaceColor',[0.8, 0.2, 0.2], 'MarkerEdgeColor',[0.8, 0.2, 0.2] );
    k = plot([a1(1) a2(1)], [a1(2) a2(2)], '-o', 'LineWidth', 2.5, 'Color', 'b',...
        'MarkerFaceColor',[0.8, 0.2, 0.2], 'MarkerEdgeColor',[0.8, 0.2, 0.2] );
    
    axis equal
    xlim([-2.5 2.5]); ylim([-2.5 2.5]);
    xlabel('x axis'); ylabel('y-axis');
    grid minor
    titleName = ['Manipulator Position: \theta_1 = ' num2str(theta_1(j)) ', \theta_2 = ' num2str(theta_2(j)) ' Snapshot #' num2str(count)];
    title(titleName);
    
    legend([h, oS, oG], 'Manipulator', 'Start', 'Goal', 'Location', 'NE')
    hold off
    
    figureName = ['Manip_Arm_Ex_2_b_' num2str(count) '.png'];
    saveas(gcf, figureName)
    
    
    pause(0.1);
    count = count + 1;
end  


%% Exercise 3.c
clear all
% wokspace
ospace= importdata("obstacle_Ex_3_c.txt");
% C-Space representation
cspace_a= importdata("cSpace_Ex_3_c.txt");
% path
path = importdata("wavefrontPath_cSpace_Ex2_c.txt");
theta_1 = path(:,1);
theta_2 = path(:,2);

% C-Space
n = sqrt(size(cspace_a,1));

collisions = flip(reshape(cspace_a(:,3), [n,n]));

% Start
thetaStart = 2*[ 180,360-0];
% Goal
thetaGoal = 2*[ 1,360-0];

cSpace_obs = imshow(collisions);

hold on;

%     % Point
%     cSpacePath(1) = plot(theta_1(j),theta_2(j), 'o', 'MarkerSize', 10,...
%         'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
% path
cSpacePath(1) = plot(2*wrapTo180(theta_1),720-2*wrapTo180(theta_2), '--b', ...
    'LineWidth', 2.5);

% start 
cS = plot(thetaStart(1), thetaStart(2), 'o', 'MarkerSize', 10,...
    'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');
% goal
cG = plot(thetaGoal(1), thetaGoal(2), 'o', 'MarkerSize', 10,...
    'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g');


xticks([1/n 0.25 0.5 0.75 1]*n);
xticklabels({'0','90','180','270','360'});
yticks([1/n 0.25 0.5 0.75 1]*n);
yticklabels({'360','270','180','90','0'});

xlabel("\theta_1 [deg]");ylabel("\theta_2 [deg]");

title('Exercise 2.c Workspace vs Configuration Space') ;
grid minor
axis on
legend([cSpacePath cS cG], 'C-Space Path', 'Start', 'Goal', 'Location', 'NE');
hold off
saveas(gcf, 'CSpacePath_Ex_2_c.png')
% Manipulator Images

% Bar info
a = [ 1; 1];
figure
count = 1;
for j = round(linspace(1,length(path),10))
    
    if j > 1
        delete(h);
        delete(k);
        delete(polyPlot);
    end
    
    for i = 1:size(ospace,1)
       vertices = ospace(i,:);
       pgon(i) = polyshape(vertices(1:2:end),vertices(2:2:end));    
    end
    
    polyPlot = plot(pgon,'FaceColor', [0.2,0.3,0.76]);
    
    a1 = [ a(1)*cosd(theta_1(j)), a(1)*sind(theta_1(j))];
    a2 = [ a1(1) + a(2)*cosd(theta_1(j)+theta_2(j)), a1(2) + a(2)*sind(theta_1(j)+theta_2(j))];
    
    hold on
    % start 
    oS = plot(-2, 0, 'o', 'MarkerSize', 10,...
        'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');
    % goal
    oG = plot(2, 0, 'o', 'MarkerSize', 10,...
        'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g');
    
    h = plot([0 a1(1)], [0 a1(2)], '-o', 'LineWidth', 1.5, 'Color', 'b',...
        'MarkerFaceColor',[0.8, 0.2, 0.2], 'MarkerEdgeColor',[0.8, 0.2, 0.2] );
    k = plot([a1(1) a2(1)], [a1(2) a2(2)], '-o', 'LineWidth', 1.5, 'Color', 'b',...
        'MarkerFaceColor',[0.8, 0.2, 0.2], 'MarkerEdgeColor',[0.8, 0.2, 0.2] );
    
    axis equal
    xlim([-2.5 2.5]); ylim([-2.5 2.5]);
    xlabel('x axis'); ylabel('y-axis');
    grid minor
    titleName = ['Manipulator Position: \theta_1 = ' num2str(theta_1(j)) ', \theta_2 = ' num2str(theta_2(j)) ' Snapshot #' num2str(count)];
    title(titleName);
    
    legend([h, oS, oG], 'Manipulator', 'Start', 'Goal', 'Location', 'NE')
    
    hold off
    
    figureName = ['Manip_Arm_Ex_2_c_' num2str(count) '.png'];
    saveas(gcf, figureName)
    
    
    pause(0.1);
    count = count + 1;
end  


