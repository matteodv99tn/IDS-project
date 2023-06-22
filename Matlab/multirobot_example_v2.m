clear;
clc;
setup;


%% --- Setup
t = 0:dt:100;


systems  = {
    System([3; 0]); ...
    System([-3; 0]); ...
    System([0; -5]); ...
    };
N_robots = length(systems);
Q = ones(N_robots) / N_robots;

obj = dataset{randi(numel(dataset))};
k_sens = 0;

for k = 1:length(t)
    cellfun(@update, systems);

    if mod(k, 300) == 0 % Perform  a scan


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

        cellfun(@(sys, F, a) sys.merge_data(map_to_add, F, a, 1), systems, F, a);

        figure(1), clf, hold on;
        cellfun(@(sys) plot(sys.manipulator), systems);
        plot(obj);
        plot(systems{1}.map);
        axis equal;
        xlim([-10, 10]);
        ylim([-10, 10]);
        plot(systems{1}.planner.allowed_region);
        plot(systems{2}.planner.allowed_region);
        plot(systems{3}.planner.allowed_region);
        grid on;
    end

end

