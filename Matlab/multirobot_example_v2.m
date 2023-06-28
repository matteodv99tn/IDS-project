clear;
clc;
setup;

%% --- Setup
t = 0:dt:100;


systems  = {
    System([3; 3]); ...
    System([2; -4]); ...
    System([-3; 3]); ...
    };

systems{1}.manipulator.set_initial_joint_config([pi/6; -4/6*pi + randn()*0.2; 2*pi*rand()]);
systems{2}.manipulator.set_initial_joint_config([-5*pi/6; -5/6*pi + randn()*0.2; 2*pi*rand()]);
systems{3}.manipulator.set_initial_joint_config([5*pi/6; 4/6*pi + randn()*0.2; 2*pi*rand()]);
N_robots = length(systems);
Q = ones(N_robots) / N_robots;

obj = dataset{randi(numel(dataset))};

obj.RF = rototranslation_matrix(-2+4*rand(), -2+4*rand(), 2*pi*rand())

k_sens = 0;

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

    if mod(k, 300) == 0 % Perform  a scan
        fprintf("----------------------------------\n");


        % --- Voronoi map update
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

        % Increased map
        map_to_add = MapEstimator(false);
        for i = 1:N_robots
            if ~isempty(newmap{i})
                map_to_add.conditional_join(newmap{i});
            end
        end

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

        cellfun(@(sys, F, a) sys.merge_data(map_to_add, F, a, 1), systems, F, a);

        figure(1), clf, hold on;
        cellfun(@(sys) plot(sys.manipulator), systems);
        plot(obj);
        plot(systems{1}.map);
        axis equal;
        XX = 7;
        xlim([-XX, XX]);
        ylim([-XX, XX]);
        plot(systems{1}.planner.allowed_region);
        plot(systems{2}.planner.allowed_region);
        plot(systems{3}.planner.allowed_region);
        grid on;


        map = systems{1}.map;
        disc = 0;
        for i = 1:map.get_size()
            if map.get_max_uncertainty(i) < config.planner.search_th
                disc = disc + 1;
            end
        end
        fprintf(" > Discovered %d/%d points\n", disc, map.get_size());

        rm_idxs = cellfun(@undesired_states, systems, "UniformOutput", false);
        rm_idxss = [];
        for i = 1:N_robots
            rm_idxss = [rm_idxss; rm_idxs{i}];
        end

        if ~isempty(rm_idxss)
            fprintf("WATH OUT\n");
        end

        redundant_states = cellfun(@(sys) sys.map.redundant_states(), systems, "UniformOutput", false);
        states_to_remove = [];
        for i = 1:N_robots
            if ~isempty(redundant_states{i})
                states_to_remove = redundant_states{i};
                break;
            end
        end
        if ~isempty(states_to_remove)
            fprintf(" === REMOVING A STATE === \n");
            cellfun(@(sys) sys.map.process_overlapping_states(states_to_remove), ...
                systems);
        end
    end

end

