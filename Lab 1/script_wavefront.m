clc;
%clear all;
close all;

load('map.mat');

tic,[value_map, trajectory] = wavefront(unnamed, [45,4], [5,150]),toc;

[m,n] = size(value_map);

%show the trajectory vector
trajectory;

%2D potential map
figure,
imagesc(value_map);
colorbar

%Show the map as an image with different potentials in 3D
figure,
%Create a surface plot for the potential matrix
surf([1:n], [1:m], value_map);
hold on
imagesc(value_map);
colorbar
title('Potential Map for wavefront');

%Display trajectory map
colormap=[1 1 1; 0 0 0; 1 0 0; 0 1 0; 0 0 1];
map_temp = map;
for i=1:size(trajectory,1)
    map_temp(trajectory(i,1), trajectory(i,2)) = 5;
end
figure,
%imagesc(map_temp);
imshow(uint8(map_temp), colormap);