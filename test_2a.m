clear;
clc;
setup;

config = get_current_configuration();


%  ____       _
% / ___|  ___| |_ _   _ _ __
% \___ \ / _ \ __| | | | '_ \
%  ___) |  __/ |_| |_| | |_) |
% |____/ \___|\__|\__,_| .__/
%                      |_|

test_name = "test_2a";
description = strcat("Problem summary:\n", ...
    "2 robots\n", ...
    "static object\n", ...
    "collision avoidance strategy: maximum size\n"...
    );
t = 0:config.simulation.dt:config.simulation.max_t;

systems  = {
    System([3; 3]); ...
    System([2; -4]); ...
    };

systems{1}.manipulator.set_initial_joint_config([pi/6; -4/6*pi + randn()*0.2; 2*pi*rand()]);
systems{2}.manipulator.set_initial_joint_config([-5*pi/6; -5/6*pi + randn()*0.2; 2*pi*rand()]);

N_robots = length(systems);
Q = ones(N_robots) / N_robots;

obj_i = randi(numel(dataset));
obj = dataset{obj_i};


%  ___       _ _   _       _ _          _   _
% |_ _|_ __ (_) |_(_) __ _| (_)______ _| |_(_) ___  _ __
%  | || '_ \| | __| |/ _` | | |_  / _` | __| |/ _ \| '_ \
%  | || | | | | |_| | (_| | | |/ / (_| | |_| | (_) | | | |
% |___|_| |_|_|\__|_|\__,_|_|_/___\__,_|\__|_|\___/|_| |_|
%

% --- Initialize folder for storing results and data logging
myfolder = fullfile("Results", test_name);
if ~exist("Results", "dir")
    mkdir("Results")
end
if ~exist(myfolder, "dir")
    mkdir(myfolder);
end
myfile = fullfile(myfolder, sprintf("simulation_%03d", numel(dir(myfolder))/2));
diary(strcat(myfile, ".log"));


k_sens = 1;
map_knowledge = zeros(config.simulation.N_meas, N_robots, 2);

fprintf("-----------------------------------------------------------------\n");
fprintf(description);
fprintf("-----------------------------------------------------------------\n");
fprintf("Spawned object: %s \n", obj.name);

%  ____  _                 _       _   _
% / ___|(_)_ __ ___  _   _| | __ _| |_(_) ___  _ __
% \___ \| | '_ ` _ \| | | | |/ _` | __| |/ _ \| '_ \
%  ___) | | | | | | | |_| | | (_| | |_| | (_) | | | |
% |____/|_|_| |_| |_|\__,_|_|\__,_|\__|_|\___/|_| |_|
%
for k = 1:length(t)

    for i = 1:N_robots
        systems{i}.manipulator.other_points_pos = {};
        for j = 1:N_robots
            if i ~= j
                systems{i}.manipulator.other_points_pos{end+1} = systems{j}.manipulator.get_voronoi_points();
            end
        end
    end
    cellfun(@update, systems);

    if mod(k, config.simulation.k_meas) == 0 % Perform  a scan

        %  ____  _     _        _ _           _           _
        % |  _ \(_)___| |_ _ __(_) |__  _   _| |_ ___  __| |
        % | | | | / __| __| '__| | '_ \| | | | __/ _ \/ _` |
        % | |_| | \__ \ |_| |  | | |_) | |_| | ||  __/ (_| |
        % |____/|_|___/\__|_|  |_|_.__/ \__,_|\__\___|\__,_|
        %
        %  ____
        % |  _ \ _ __ ___   ___ ___  ___ ___
        % | |_) | '__/ _ \ / __/ _ \/ __/ __|
        % |  __/| | | (_) | (_|  __/\__ \__ \
        % |_|   |_|  \___/ \___\___||___/___/
        %
        fprintf(" -------------- Sensors Step %d --------------\n", k_sens);
        k_sens = k_sens + 1;

        % --- Update allowed region
        voronoi_points = zeros(3*N_robots, 2);
        for i = 1:N_robots
            voronoi_points(3*i-2:3*i, :) = systems{i}.manipulator.get_voronoi_points();
        end
        [V, R] = ClippedVoronoi(voronoi_points);
        for i = 1:N_robots
            p1 = polyshape(V(R{3*i-2},:));
            p2 = polyshape(V(R{3*i-1},:));
            p3 = polyshape(V(R{3*i},:));
            ptmp = union(p1, p2);
            poly = union(ptmp, p3);
            systems{i}.planner.allowed_region = poly;
        end


        % --- Perform scan  and update
        [newmap, F, a] = cellfun(@(sys) sys.scan_object(obj), systems, 'UniformOutput', false);

        % Linear consensus
        [F, a] = linear_consensus(F, a, Q);

        % States to add to each robot
        map_to_add = MapEstimator(false);
        for i = 1:N_robots
            if ~isempty(newmap{i})
                map_to_add.conditional_join(newmap{i});
            end
        end

        % Restore values after linear consensus and improve map
        cellfun(@(sys, F, a) sys.merge_data(map_to_add, F, a, 1), systems, F, a);

        % Remove to the voronoi cell the envelope of the other manipulators
        for i = 1:N_robots
            systems{i}.constraint_voronoi_cell();
            for j = 1:N_robots
                if i ~= j
                    rob_points = systems{j}.manipulator.get_voronoi_points();
                    rob_envelope = polybuffer(rob_points, 'line', 0.25);
                    systems{i}.planner.allowed_region = subtract(systems{i}.planner.allowed_region, rob_envelope);
                end
            end
        end


        % --- Compute the number of fully detected points
        for i = 1:N_robots
            map = systems{i}.map;
            n_discovered = 0;
            for j = 1:map.get_size()
                if map.get_max_uncertainty(j) < config.planner.search_th
                    n_discovered = n_discovered + 1;
                end
            end
            map_knowledge(k_sens, i, 1) = n_discovered;
            map_knowledge(k_sens, i, 2) = map.get_size();
            fprintf("Robot #%d: %d/%d discovered\n", i, n_discovered, map.get_size());
        end


        % --- TODO: check this part of the code
        rm_idxs = cellfun(@undesired_states, systems, "UniformOutput", false);
        rm_idxss = [];
        for i = 1:N_robots
            rm_idxss = [rm_idxss; rm_idxs{i}];
        end
        if ~isempty(rm_idxss)
            fprintf("WATH OUT\n");
        end

        % --- Check for overlapping states
        redundant_states = cellfun(@(sys) sys.map.redundant_states(), systems, "UniformOutput", false);
        states_to_remove = [];
        for i = 1:N_robots
            if ~isempty(redundant_states{i})
                states_to_remove = redundant_states{i};
                break;
            end
        end
        if ~isempty(states_to_remove)
            fprintf("> Joining 2 overlapping states\n");
            cellfun(@(sys) sys.map.process_overlapping_states(states_to_remove), ...
                systems);
        end



        %   ____ _               _  __ _           _   _
        %  / ___| | __ _ ___ ___(_)/ _(_) ___ __ _| |_(_) ___  _ __
        % | |   | |/ _` / __/ __| | |_| |/ __/ _` | __| |/ _ \| '_ \
        % | |___| | (_| \__ \__ \ |  _| | (_| (_| | |_| | (_) | | | |
        %  \____|_|\__,_|___/___/_|_| |_|\___\__,_|\__|_|\___/|_| |_|
        %
        for i = 1:N_robots
            if systems{i}.map.get_size() >= 3
                % --- Get the best fit
                [idx, params, costs] = systems{i}.map.find_best_fit(dataset);
                fprintf("Robot #%d guess: %s\n", i, dataset{idx}.name);

                % --- Based on the guess, check if some states might be removed
                idx = systems{i}.map.find_removable_state(params, dataset{idx});
                if ~isempty(idx)
                    fprintf(" > Removing a state based on the guess\n");
                    for j = 1:N_robots
                        systems{j}.map.remove_state_i(idx);
                    end
                end
            end
        end

        cellfun(@(sys) sys.planner.save_region(), systems);
        %  ____  _       _
        % |  _ \| | ___ | |_
        % | |_) | |/ _ \| __|
        % |  __/| | (_) | |_
        % |_|   |_|\___/ \__|
        %
        figure(1), clf, hold on
        ttl = sprintf("Time: %.2fs - %.1f%%", k*config.simulation.dt, k/length(t)*100);
        title(ttl);
        cellfun(@(sys) plot(sys.manipulator), systems);
        plot(obj);
        plot(systems{1}.map);
        axis equal;
        XX = 4;
        xlim([-XX, XX]);
        ylim([-XX, XX]);
        for i = 1:N_robots
            plot(systems{i}.planner.allowed_region);
        end
        grid on;
    end
end



%  ____  _
% / ___|| |_ ___  _ __ ___
% \___ \| __/ _ \| '__/ _ \
%  ___) | || (_) | | |  __/
% |____/ \__\___/|_|  \___|
%
fprintf("-----------------------------------------------------------------\n");
fprintf(description);
diary("off");
save(strcat(myfile, ".mat"));
