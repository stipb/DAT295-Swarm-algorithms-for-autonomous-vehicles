clc, clear all, close all
% map = binaryOccupancyMap(1000,100,10);
% setOccupancy(map, ones(10000,1000))
% x = 1:1:10000;
% y = 40*ones(1,10000)';
% setOccupancy(map,[x' y], zeros(10000,1))
image = imread('map_large_sq.png');
grayimage = rgb2gray(image);
map = binaryOccupancyMap(grayimage,3);


figure
show(map)
fig=gcf;
fig.Units='normalized';
fig.OuterPosition=[0 0 1 1];
save map_v2