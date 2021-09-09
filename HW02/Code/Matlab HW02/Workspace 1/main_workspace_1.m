

close all;
clear all



% read map data
mapFile = "mapDim.txt";
mapData = importdata(mapFile);
xLimits = mapData(1,:);
yLimits = mapData(2,:);
start = mapData(3,:);
goal = mapData(4,:);

% read obstacle file and plot map
obstacleFile = "obstacleFile.txt";
obsData = importdata(obstacleFile);

% get route
routeFile = ["routeFileBug1.txt","routeFileBug2.txt"];
%title
titles = {'Workspace 1 - Bug 1', 'Workspace 1 - Bug 2'};

for i =1:2

    route = importdata(routeFile(i));

    figure()
    hold on;



    for j = 1:size(obsData,1)
        width = obsData(j, 3) - obsData(j,1);
        height = obsData(j, 8) - obsData(j,2);

        rectangle('Position',[obsData(j, 1) obsData(j, 2) width height],...
            'FaceColor',[0 .5 .5], 'EdgeColor', [0 .5 .5])


    end



    % plot route
    plot(route(:,1), route(:,2), 'o')

    % plot start and goal
    plot(start(1), start(2), 'o', 'MarkerSize', 15,'MarkerFaceColor', 'r')
    plot(goal(1), goal(2), 'o', 'MarkerSize', 15, 'MarkerFaceColor', 'g')
    title(titles{i})
    grid minor;
    xlim(xLimits); ylim(yLimits)
    
    % Print Metrics
    fprintf('Max Distance Traveled by Bug %d on Workspace 2: %0.3f \n', i, route(end,3))
    
end

