close all;
clear;
clc;
addpath("Functions/")


points = 10 * rand(20,2);
x = points(:,1);
y = points(:,2);

DT = delaunayTriangulation(points);
triplot(DT)
F = freeBoundary(DT)

hold on;
plot(x(F), y(F), 'r--');
