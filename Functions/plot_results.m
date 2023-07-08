function plot_results(filename, plot_animation, save_plots)

    plot_rate = 15;

    %     _                      ____                              _
    %    / \   _ __ __ _ ___    |  _ \ _ __ ___   ___ ___  ___ ___(_)_ __   __ _
    %   / _ \ | '__/ _` / __|   | |_) | '__/ _ \ / __/ _ \/ __/ __| | '_ \ / _` |
    %  / ___ \| | | (_| \__ \_  |  __/| | | (_) | (_|  __/\__ \__ \ | | | | (_| |
    % /_/   \_\_|  \__, |___(_) |_|   |_|  \___/ \___\___||___/___/_|_| |_|\__, |
    %              |___/                                                   |___/
    if nargin < 2
        plot_animation = false;
    end
    if nargin < 3
        save_plots = false;
    end

    if contains(filename, ".mat")
        mat_file = filename;
        log_file = strrep(filename, ".mat", ".log");
    elseif contains(filename, ".log")
        log_file = filename;
        mat_file = strrep(filename, ".log", ".mat");
    else
        log_file = strcat(filename, ".log");
        mat_file = strcat(filename, ".mat");
    end

    fprintf(fileread(log_file));
    load(mat_file);

    plot_dt = 1 / plot_rate;
    plot_times = 0:plot_dt:config.simulation.max_t;
    plot_times_red = 0:plot_dt*config.simulation.k_meas:config.simulation.max_t;
    DT = config.simulation.dt*config.simulation.k_meas;
    meas_times = 0:DT:config.simulation.max_t;


    %  ____  _        _   _        ____  _       _
    % / ___|| |_ __ _| |_(_) ___  |  _ \| | ___ | |_ ___
    % \___ \| __/ _` | __| |/ __| | |_) | |/ _ \| __/ __|
    %  ___) | || (_| | |_| | (__  |  __/| | (_) | |_\__ \
    % |____/ \__\__,_|\__|_|\___| |_|   |_|\___/ \__|___/
    %

    figure(1), clf, hold on;
    title("Discovered states");
    knowledge_plot(meas_times, map_knowledge);
    legend("Discovered states", "Fully known states", "Location", "SouthEast");
    xlabel("Time [s]");
    ylabel("Vertex count");
    grid on;
    if save_plots
        export_figure(mat_file, "disc_states");
    end


    figure(2), clf, hold on;
    title("Detection");
    detection_plot(meas_times, detect_hist, obj_i);
    xlabel("Time [s]");
    ylabel("Detection");
    if save_plots
        export_figure(mat_file, "detection");
    end



    %  ____                              _        ____  _       _
    % |  _ \ _   _ _ __   __ _ _ __ ___ (_) ___  |  _ \| | ___ | |_ ___
    % | | | | | | | '_ \ / _` | '_ ` _ \| |/ __| | |_) | |/ _ \| __/ __|
    % | |_| | |_| | | | | (_| | | | | | | | (__  |  __/| | (_) | |_\__ \
    % |____/ \__, |_| |_|\__,_|_| |_| |_|_|\___| |_|   |_|\___/ \__|___/
    %        |___/
    if plot_animation
    close all;
    for k = 1:length(plot_times)
        tic;

        k_sensor = 1 + floor(plot_times(k) / (config.simulation.dt*config.simulation.k_meas));
        k_robot = 1 + floor(plot_times(k) / config.simulation.dt);


        figure(100), clf, hold on;
        ttl = sprintf("t = %.2f", plot_times(k));
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
    end
end
