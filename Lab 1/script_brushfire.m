clc;
clear all;
close all;

load('map.mat');

[value_map] = brushfire(unnamed);

[m,n] = size(value_map);

%Show the map as an image with different potentials
figure
%Create a surface plot for the potential matrix
surf([1:n], [1:m], value_map)
hold on
imagesc(value_map)
colorbar
title('Potential Map of brushfire algorithm');

