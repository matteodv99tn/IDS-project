% close all;
clear;
clc;

global fig_idx;
fig_idx = 0;

%% --- Setup
dt  = 0.1;
t   = 0:dt:10;
N   = length(t);

N_robots = 2;
robots      = cell(1,N_robots);
for i = 1:N_robots 
    robots{i}           = Robot(N, dt);
    robots{i}.R_odo     = 0.05 * diag(rand(2,1)) * dt;
    robots{i}.R_gps     = 0.1 * diag(rand(2,1));
    robots{i}.P_hat     = 1 * eye(3);
    robots{i}.x         = 10  * rand(3,1);
    robots{i}.x_hat     = robots{i}.x;
end

make_pd = @(x) x*x';
XALL = [];

%% --- Simulation
Q = ones(3, 3) / 3;

x_est = zeros(2, 1);
X_est = zeros(2, N);
P_est = 1 * diag([1, 1]);
P_hist = zeros(2, 2, N);

pt = [0; 0];



for k = 1:N
    cellfun(@step_time, robots);
    % cellfun(@(x) x.set_gps_uncertainty(make_pd(0.01*rand(2,2))), robots);

    cellfun(@(x) x.KF_prediction_step(randn(2,1)), robots);
    cellfun(@(x) x.KF_update_step(), robots); 

    [Fi, ai] = cellfun( ...
                @(x) x.build_composite_informations(pt), ...
                robots, ...
                'UniformOutput', false ...
                );

    Fi_sum = zeros(2, 2);
    ai_sum = zeros(2, 1);
    for i = 1:N_robots 
        Fi_sum = Fi_sum + Fi{i};
        ai_sum = ai_sum + ai{i};
    end
    Fi_mean = Fi_sum / N_robots;
    ai_mean = ai_sum / N_robots;

    % x_est = P_est*(P_est*x_est - N_robots*ai_mean);
    % P_est = inv(P_est + N_robots*Fi_mean);
    x_est = P_est*(P_est*x_est - ai_mean);
    P_est = inv(P_est + Fi_mean);

    X_est(:, k) = x_est;
    P_hist(:, :, k) = P_est;
end

Z = cellfun(...
        @(x) x.build_composite_informations([0; 0]), ...
        robots, ...
        'UniformOutput', false ...
        );
rob = robots{1};

new_figure();
err = rob.X_hat - rob.X;
err(3, :) = wrapToPi(err(3, :));

titles = {"x", "y", "\theta"};
for idx = 1:3
    subplot(3, 1, idx), hold on;
    x_unc = [t, fliplr(t)];
    y_unc = 3 * [rob.Sigma(idx, :), fliplr(-rob.Sigma(idx, :))];
    fill(x_unc, y_unc, 'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    plot(t, err(idx,:), "LineWidth", 2);
    title(titles{idx});
    grid on;
end

new_figure();
err = X_est - pt;
unc = squeeze([ ...
    sqrt(P_hist(1, 1, :)), ...
    sqrt(P_hist(2, 2, :)) ...
    ]);

titles = {"x", "y", };
for idx = 1:2
    subplot(2, 1, idx), hold on;
    x_unc = [t, fliplr(t)];
    y_unc = 3 * [unc(idx, :), fliplr(-unc(idx, :))];
    fill(x_unc, y_unc, 'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    plot(t, err(idx,:), "LineWidth", 2);
    title(titles{idx});
    grid on;
end


function new_figure()
    global fig_idx;
    fig_idx = fig_idx + 1;
    figure(fig_idx), hold on, clf;
end
