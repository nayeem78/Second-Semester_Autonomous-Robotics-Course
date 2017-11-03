clear;
clc;
close all;
%%
%load maze.mat
load map.mat
colormap=[1 1 1; 0 0 0; 1 0 0; 0 1 0; 0 0 1]; imshow(uint8(map),colormap)
hold on

[vertices,edges,path]=rrt(map,[70,80],[615,707],10000,50,0.3); %map
%[vertices,edges,path]=rrt(map,[198,206],[612,416],10000,50,0.3); %maze

% Plot all the vertices
for i = 1 : size(vertices,1)
    plot(vertices(i,2),vertices(i,1),'g+');
end

% Plot all the edges
for i = 1 : size(edges,1)
    plot([vertices(edges(i,1),2), vertices(edges(i,2),2)],[vertices(edges(i,1),1), vertices(edges(i,2),1)],'b');
end
% Plot path
for i = 1 : length(path)-1
    plot([vertices(path(i),2),vertices(path(i + 1),2)],[vertices(path(i),1), vertices(path(i + 1), 1)],'r-','LineWidth',1.5);
end

% % Smooth the path 
[path_smooth]=smooth(map,path,vertices,3);
for i = 1 : length(path_smooth)-1
    plot([vertices(path_smooth(i),2),vertices(path_smooth(i + 1),2)],[vertices(path_smooth(i),1), vertices(path_smooth(i + 1), 1)],'k-','LineWidth',1.5);
end

hold off