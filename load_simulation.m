clear;
clc;


%  ____       _
% / ___|  ___| |_ _   _ _ __
% \___ \ / _ \ __| | | | "_ \
%  ___) |  __/ |_| |_| | |_) |
% |____/ \___|\__|\__,_| .__/
%                      |_|

test_name = "example";
test_num = 1;

plot_rate = 15;     % refresh rate of  the dynamic plot

%  ___       _ _   _       _ _          _   _
% |_ _|_ __ (_) |_(_) __ _| (_)______ _| |_(_) ___  _ __
%  | || "_ \| | __| |/ _` | | |_  / _` | __| |/ _ \| "_ \
%  | || | | | | |_| | (_| | | |/ / (_| | |_| | (_) | | | |
% |___|_| |_|_|\__|_|\__,_|_|_/___\__,_|\__|_|\___/|_| |_|
%

% --- Reading back data
file_name = fullfile("Results", test_name, sprintf("simulation_%03d", test_num));
if ~exist(strcat(file_name, ".mat"), "file")
    error("File does not exist");
end

load(strcat(file_name, ".mat"));
fprintf(fileread(strcat(file_name, ".log")));


% --- Dynamic plot
plot_dt = 1 / plot_rate;
plot_times = 0:plot_dt:config.simulation.max_t;

%  ____  _        _   _        ____  _       _
% / ___|| |_ __ _| |_(_) ___  |  _ \| | ___ | |_ ___
% \___ \| __/ _` | __| |/ __| | |_) | |/ _ \| __/ __|
%  ___) | || (_| | |_| | (__  |  __/| | (_) | |_\__ \
% |____/ \__\__,_|\__|_|\___| |_|   |_|\___/ \__|___/
%







%  ____                              _        ____  _       _
% |  _ \ _   _ _ __   __ _ _ __ ___ (_) ___  |  _ \| | ___ | |_ ___
% | | | | | | | "_ \ / _` | "_ ` _ \| |/ __| | |_) | |/ _ \| __/ __|
% | |_| | |_| | | | | (_| | | | | | | | (__  |  __/| | (_) | |_\__ \
% |____/ \__, |_| |_|\__,_|_| |_| |_|_|\___| |_|   |_|\___/ \__|___/
%        |___/

for k = 1:length(plot_times)

    tic;
    k_sensor = 1+floor(plot_times(k) / (config.simulation.dt * config.simulation.k_meas));
    k_robot = 1+floor(plot_times(k) / config.simulation.dt);

    figure(100), clf, hold on;
    ttl = sprintf("Time %.2f s", plot_times(k));
    title(ttl);
    plot(obj);
    cellfun(@(sys) plot_manipulator(sys.manipulator, k_robot), systems);
    cellfun(@(sys) plot_voronoi_cell(sys.planner, k_sensor), systems);
    cellfun(@(sys) plot_measurement(sys.map, k_sensor), systems);
    plot_map(systems{1}.map, k_sensor);
    axis equal;

    delta_plot = toc;
    pause(plot_dt - delta_plot);
end



%  _____                 _   _
% |  ___|   _ _ __   ___| |_(_) ___  _ __  ___
% | |_ | | | | "_ \ / __| __| |/ _ \| "_ \/ __|
% |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
% |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
%

function plot_manipulator(man, time)

    L1 = man.L1;
    L2 = man.L2;
    q  = man.q_true_hist(time, :);
    OO = [0; 0; 1];

    RF0 = translation_matrix(man.origin);
    RF1 = RF0 * rotation_matrix(q(1));
    RF2 = RF1 * translation_matrix([L1, 0, 0]) * rotation_matrix(q(2));
    RF3 = RF2 * translation_matrix([L2, 0, 0]) * rotation_matrix(q(3));

    P1 = RF1 * OO;
    P2 = RF2 * OO;
    P3 = RF3 * OO;
    aa = RF3 * [1; 0; 0];
    points = [P1, P2, P3];

    plot(P1(1), P1(2), "kx", "LineWidth", 2, "MarkerSize", 15);
    plot(points(1, :), points(2, :), "-ok", "LineWidth", 2, "MarkerSize", 10);
    quiver(P3(1), P3(2), aa(1), aa(2), "b", "LineWidth", 2, "MaxHeadSize", 0.5);

end % plot_manipulator


function plot_voronoi_cell(planner, time)
    if isempty(planner.allreg_hist{time})
        time = time - 1;
    end
    plot(planner.allreg_hist{time});
end % plot_voronoi_cell


function plot_map(map, time)
    if isempty(map.x_hist{time}) && time > 1
        time = time - 1;
    end
    x = map.x_hist{time};
    P = map.P_hist{time};

    for i = 1:numel(x)/2
        xi = x(2*i-1:2*i);
        Pi = P(2*i-1:2*i, 2*i-1:2*i);
        [data_x, data_y] = uncertainty_ellipsoid(xi, Pi);
        plot(data_x, data_y, "b", "LineWidth", 1);
    end
end % plot_map


function plot_measurement(map, time)
    z = map.z_hist{time};
    R = map.R_hist{time};

    for i = 1:numel(z)/2
        zi = z(2*i-1:2*i);
        Ri = R(2*i-1:2*i, 2*i-1:2*i);
        [data_x, data_y] = uncertainty_ellipsoid(zi, Ri);
        plot(data_x, data_y, "r:", "LineWidth", 1);
    end
end % plot_measurement
