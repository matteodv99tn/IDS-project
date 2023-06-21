%
%   EXAMPLE
% 
% In a nutshell
% - multiple unicycles are allowed to move in space. Their position w.r.t. an 
%   inertial frame is perfectly known (no uncertainty);
% - each robot can take a relative measurement of a point of interest in the 
%   space
%


%% --- Startup
clear;
clc;

%% --- Configuration
Lx_min = -10;       % This values are defining the region in which robots and
Lx_max =  10;       % points are allowed to spawn
Ly_min = -10;
Ly_max =  10;

N_robots = 2;       % N. of robots to generate
N_points = 1;       % N. of points of interest to estimate

robots = cell(N_robots, 1); 
points = cell(N_points, 1);
%   Implementation note:
% Each cell in "robots" is a struct that contains:
% - "x": the 3D vector describing the unicycle (x, y, theta);
% - "R": the covariance matrix that's used in the point estimation;
% Each cell in "points" is a struct that contains:
% - "position": the true position of the point;
% - "x": the state estimate of the point (initially set to "position" for 
%   convenience;
% - "P": the covariance matrix on the estimate (initially "big enough").


%% --- Initialization

LB = [Lx_min; Ly_min; 0];   % lower bound for robot position
Delta = [Lx_max - Lx_min; Ly_max - Ly_min; 2*pi];
for i = 1:N_robots % initialize robot position
    robots{i}.x = LB + Delta.*rand(3, 1);
    tmp = rand(2, 2);
    robots{i}.R = tmp*tmp';
end
for i = 1:N_points % initialize point position
    points{i}.position = LB + Delta.*rand(3, 1);
    points{i}.position(3) = 1;
    points{i}.x = points{i}.position(1:2);
    points{i}.P = 100*eye(2);
end



figure(1), clf, hold on;
cellfun(@draw_unicycle, robots);
cellfun(@draw_point, points);
xlim([Lx_min, Lx_max]);
ylim([Ly_min, Ly_max]);
legend();
axis equal;
grid on;





%% --- Custom functions
function M0f = unicycle_RF(X)
    % Homogeneous transformation matrix that project points defined in the 
    % unicycles RF to ground coordinates
    % "X" is a vector of the unicycle state (x, y, theta)
    x = X(1);
    y = X(2);
    theta = X(3);
    
    M0f = [cos(theta), -sin(theta), x; ...
           sin(theta),  cos(theta), y; ...
           0,           0,          1 ];
end % unicycle_RF function
        

function draw_unicycle(uni)
    % Draws the unicycle 
    M01 = unicycle_RF(uni.x);
    pts = [1,   -0.6,   -0.6,   1; ...
           0,   0.5,    -0.5,   0; ...
           1,   1,      1,      1];
    p = M01 * pts;
    plot(p(1,:), p(2,:), "k-", "DisplayName", "");
    plot(uni.x(1), uni.x(2), "ok", "DisplayName", "Robot");
end


function draw_point(p)
    % Draws a point struct
    plot(p.position(1), p.position(2), "sk");
    [x, y] = uncertainty_ellipsoid(p.x, p.P);
    plot(x, y, "--k", "DisplayName", "Ellipsoid");
end


function [x, y] = uncertainty_ellipsoid(mu, S)
    theta = linspace(0, 2*pi, 101);
    circle = 3 * [cos(theta); sin(theta)];
    [eig_vect, eig_vals] = eig(S);
    data = eig_vect * sqrt(eig_vals) * circle;
    x = data(1, :) + mu(1);
    y = data(2, :) + mu(2);
end
