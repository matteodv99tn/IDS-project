clear;
clc;
setup;


%% --- Setup
t = 0:dt:100;


systems  = {
    System([3; 0]); ...
    System([-3; 0]); ...
    System([0, -7]); ...
    };
N_robots = length(systems);
Q = ones(N_robots) / N_robots;

obj = dataset{randi(numel(dataset))};
k_sens = 0;

for k = 1:length(t)
    cellfun(@update, systems);

    if mod(k, 300) == 0 % Perform  a scan


        % --- Voronoi map update
        EE_positions = zeros(N_robots, 2);
        for i = 1:N_robots
            tmp = systems{i}.manipulator.get_EE_state(true);
            EE_positions(i, :) = tmp(1:2);
        end
        [V, R] = ClippedVoronoi(EE_positions);

        % for i = 1:N_robots
        %     poly = V([R{i}, R{i}(1)], :);
        %     systems{i}.planner.allowed_region = poly;
        % end



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

        cellfun(@(sys, F, a) sys.merge_data(map_to_add, F, a, 1), systems, F, a);

        figure(1), clf, hold on;
        cellfun(@(sys) plot(sys.manipulator), systems);
        plot(obj);
        plot(systems{1}.map);
        voronoi(EE_positions(:, 1), EE_positions(:, 2));
    end

end

