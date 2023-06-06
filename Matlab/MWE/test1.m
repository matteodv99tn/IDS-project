%
% In nutshell:
% - multiples vehicles robots (modelled as unicycles) are allowed to move in 
%   the space. Their position is fully known (no uncertainty).
%   They are controlled (almost) randomly.
% - the robots are sensing the same point of interest.
%
% The robot have a shared absolute knowledge of the world reference frame and
% measurement are defined in such RF.
% Here we consider a full interconnection between robots to have the best
% message exchange possible.
%
% Note: in this script I make extensive use of the "cellfun" function to 
% have a less pedantic notation; for more information take a look here:
%   https://it.mathworks.com/help/matlab/ref/cellfun.html
%
clear;
clc;

addpath("functions");
addpath("classes");


%  ____       _               
% / ___|  ___| |_ _   _ _ __  
% \___ \ / _ \ __| | | | '_ \ 
%  ___) |  __/ |_| |_| | |_) |
% |____/ \___|\__|\__,_| .__/ 
%                      |_|    
%% --- Configuration
N_time = 100;
N_robots = 5;


%% --- Initialization
t = 0:N_time;

robots = cell(N_robots, 1);
for i = 1:N_robots
    robots{i} = Robot(1);
end
point = [-10; -10] + 20*rand(2,1);

Q = ones(N_robots) / N_robots;

x_central = zeros(2,1);
P_central = 20*eye(2);
H = cell(N_robots, 1);
H(:) = {eye(2)};
H = vertcat(H{:});

est_err = zeros(2, N_time);
cov_err = zeros(2, N_time);
est_err2 = zeros(2, N_time);
cov_err2 = zeros(2, N_time);


%%
%  ____  _                 _       _   _             
% / ___|(_)_ __ ___  _   _| | __ _| |_(_) ___  _ __  
% \___ \| | '_ ` _ \| | | | |/ _` | __| |/ _ \| '_ \ 
%  ___) | | | | | | | |_| | | (_| | |_| | (_) | | | |
% |____/|_|_| |_| |_|\__,_|_|\__,_|\__|_|\___/|_| |_|
%                                                    
for k = 1:N_time

    % --- Move the robot 
    cellfun(@move, robots);

    % --- Distributed KF
    % Build the measurement and associated covariance matrices
    [z, R] = cellfun(@(r) r.get_measurement(point), robots, ...
                     "UniformOutput", false);
    % Build the composite informations
    [Fi, ai] = cellfun(@(r, z) r.build_composite_informations(z), ...
                       robots, z, ...
                       "UniformOutput", false);
    % Apply the linear consensus algorithm to share the knowledge
    [Fi, ai] = linear_consensus(Fi, ai, Q);
    % Perform the KF update step on each robot with the corresponding composite
    % informations
    cellfun(@(r, F, a) r.distributed_KF_update(F, a, N_robots), ...
            robots, Fi, ai);
    
    % --- Centralized KF
    Z = vertcat(z{:});          % vector of all measurements
    RR = blkdiag(R{:});         % covariance matrix of the ensamble of meas.
    S = H*P_central*H' + RR;
    W = P_central*H'*inv(S);
    x_central = x_central + W*(Z - H*x_central);
    P_central = (eye(2) - W*H)*P_central;

    % --- Retrieve data of the first robot to plot the estimation ellipsoid
    X = robots{1}.x_est;
    P = robots{1}.P_est;
    [x, y] = uncertainty_ellipsoid(X, P);

    % --- Plot
    figure(1), clf, hold on;
    cellfun(@(r, z) plot(r, z), robots, z);
    plot(point(1), point(2), "sk");
    plot(x, y, "-g");
    plot(X(1), X(2), "g*");
    xlim([-15 15]);
    ylim([-15 15]);
    axis equal;
    grid on;
    
    % --- Store the error at the current iteration timestamp
    est_err(:, k) = robots{1}.x_est - point;
    cov_err(:, k) = sqrt(diag(robots{1}.P_est));
    est_err2(:, k) = x_central - point;
    cov_err2(:, k) = sqrt(diag(P_central));
end

%%
%  ____  _       _       
% |  _ \| | ___ | |_ ___ 
% | |_) | |/ _ \| __/ __|
% |  __/| | (_) | |_\__ \
% |_|   |_|\___/ \__|___/
%                        

% --- Plot the error of the distributed KF
figure(2), clf, hold on;
subplot(2, 1, 1), hold on;
title("Distributed KF - Robot 1");
plot(est_err(1,:), "b", "LineWidth", 3);
plot(3*cov_err(1,:), "b--");
plot(-3*cov_err(1,:), "b--");
ylabel("x error");
grid on;

subplot(2, 1, 2), hold on;
plot(est_err(2,:), "b", "LineWidth", 3);
plot(3*cov_err(2,:), "b--");
plot(-3*cov_err(2,:), "b--");
ylabel("y error");
grid on;

% --- Plot the error of the centralized KF
figure(3), clf, hold on;
subplot(2, 1, 1), hold on;
title("Centralized KF - Robot 1");
plot(est_err2(1,:), "b", "LineWidth", 3);
plot(3*cov_err2(1,:), "b--");
plot(-3*cov_err2(1,:), "b--");
ylabel("x error");
grid on;

subplot(2, 1, 2), hold on;
plot(est_err2(2,:), "b", "LineWidth", 3);
plot(3*cov_err2(2,:), "b--");
plot(-3*cov_err2(2,:), "b--");
ylabel("y error");
grid on;
