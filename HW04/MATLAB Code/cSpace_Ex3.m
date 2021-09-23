%% HW04 MATLAB Visualization Code


%% House Keeping
clc; close all; clear all;

%% Exercise 3.a
ospace= importdata("obstacle_Ex_3_a.txt");



figure

hold on
subplot(1,2,1)
for i = 1:size(ospace,1)
   vertices = ospace(i,:);
   pgon = polyshape(vertices(1:2:end),vertices(2:2:end));
   plot(pgon)
end
axis equal
xlim([-2 2]); ylim([-2 2]);
xlabel('x axis'); ylabel('y-axis');
grid minor
hold off

cspace_a= importdata("cSpace_Ex_3_a.txt");

n = sqrt(size(cspace_a,1));

collisions = flip(reshape(cspace_a(:,3), [n,n]));
subplot(1,2,2)

imshow(collisions)
grid minor
axis on

xticks([1/n 0.25 0.5 0.75 1]*n)
xticklabels({'0','90','180','270','360'})
yticks([1/n 0.25 0.5 0.75 1]*n)
yticklabels({'0','90','180','270','360'})

xlabel("\theta_1 [deg]");ylabel("\theta_2 [deg]")

sgtitle('Exercise 3.a Workspace vs Configuration Space') 
% theta_1 = reshape(cspace_a(:,1),[n,n]);
% theta_2 = reshape(cspace_a(:,2),[n,n]);
% collisions = (reshape(cspace_a(:,3), [n,n]));
% surf(theta_1, theta_2, collisions)
% for i = 1:n
%     for j = 1:n
%         
%         X = 
%         
%     end
% end





