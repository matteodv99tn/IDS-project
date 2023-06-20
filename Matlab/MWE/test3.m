close all; clear; clc;
rng default;

x = [0; 1; 5; 2];
y = [0; 2; 5; -1];
X = [x, y];


bound = [-20 20 -20 20];


tic
[vx, vy, vxclip, vyclip] = VoronoiLimitRectSquare(x, y, bound, 1, 1);
toc
