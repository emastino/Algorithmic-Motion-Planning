%% HW04 MATLAB Visualization Code


%% House Keeping
clc; close all; clear all;

%% Exercise 1

cspace= importdata("cSpaceFile.txt");

% all angles
angles = cspace(:,3);

% unique angles
b = DedupSequence(angles);

figure
hold on
for i = 1:length(b)
   vertices = cspace(find((cspace(:,3)==b(i))), :);
   fill3(vertices(:,1),vertices(:,2),vertices(:,3), [0+i/length(b), 1-i/length(b), 0.3]); 
end
grid minor

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


%% Exercise 2