classdef MotionPlanner < handle

properties %% ---- Attributes of the class --------------------------------------------------------
    home_config = pi/2 * ones(3, 1);
    idx_to_follow = 0;
    robot     = Manipulator(5,5);        % initialize the robot and the camera
    cam     = Camera(60*pi/180, 201);

    allowed_region = [];
    target = [];
    repulsion_points = [];
    attraction_points = [];

    prev_target = [];
    target_count = 0;

    wrong_state_pos = [];

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = MotionPlanner()
    end % MotionPlanner constructor


    function update_target(self, manipulator,  map)

        xEE = manipulator.get_EE_state(true);
        if isempty(self.target)
            omega = 1;
            dir = self.build_repulsive_direction(manipulator);
            if norm(dir) > 0.0001
                dir = dir/norm(dir) * 0.5;
            end
        elseif ~isinterior(self.allowed_region, xEE(1), xEE(2))
            fprintf("EE not in cell!\n");
            [d, xr, yr] = p_poly_dist(xEE(1), xEE(2), self.allowed_region.Vertices(:,1), self.allowed_region.Vertices(:,2));
            dir = [xr; yr];
            dir = dir/norm(dir) * 5;
            omega = 0;
        else
            xEE = manipulator.get_EE_state(true);
            dir_target = self.target - xEE(1:2);
            dir_target = dir_target / norm(dir_target);
            [dir_repulsive, d_min] = self.build_repulsive_direction(manipulator);
            if norm(dir_repulsive) > 0.001
                dir_repulsive = dir_repulsive/norm(dir_repulsive) * 0.1;
                dir_target = dir_target + self.weight(d_min)*dir_repulsive;
            end

            incell = false;
            i = 0;
            [d, xr, yr] = p_poly_dist(xEE(1), xEE(2), self.allowed_region.Vertices(:,1), self.allowed_region.Vertices(:,2));
            dir_repulsive = [xr; yr];
            dir_repulsive = dir_repulsive/norm(dir_repulsive) * 0.1;
            while ~incell
                dir = dir_target + i*dir_repulsive;
                dir = dir/norm(dir) * 0.5;
                x_EE_next = xEE(1:2) + 20*dir*manipulator.dt;
                incell = isinterior(self.allowed_region, x_EE_next(1), x_EE_next(2));
                i = i + 1;
                if i > 100
                    fprintf("STUCK!\n");
                    dir = [0; 0];
                    incell = true;
                end
            end
            alpha = atan2(dir_target(2), dir_target(1));
            gamma = mod(xEE(3) - alpha + pi, 2*pi) - pi;
            omega = -gamma*2;
        end
        manipulator.controller.set_target([dir; omega]);
    end


    function plan_motion(self, manipulator, map)
        self.prev_target = self.target;

        config = get_current_configuration();
        x_EE = manipulator.get_EE_state(true);


        self.repulsion_points = [];
        self.attraction_points = [];
        for i = 1:map.get_size();
            eps = map.get_max_uncertainty(i);
            if eps < config.planner.search_th
                self.repulsion_points = [self.repulsion_points, map.get_state_i(i)];
            else
                self.attraction_points = [self.attraction_points, map.get_state_i(i)];
            end
        end

        if ~isempty(self.attraction_points)
            feas_points = isinterior(self.allowed_region, ...
                                      self.attraction_points(1,:), ...
                                      self.attraction_points(2,:));

            feas_points = self.attraction_points(:, feas_points);
            if ~isempty(feas_points)
                [cx, cy] = centroid(self.allowed_region);
                centr = x_EE(1:2);
                dist = vecnorm(feas_points - centr);
                [~, idx] = min(dist);
                self.target = feas_points(:,idx);
                fprintf("Setting feasible point\n");
            else
                xEE = x_EE(1:2);
                dist = vecnorm(self.attraction_points - xEE);
                [~, idx] = min(dist);
                self.target = self.attraction_points(:,idx);
                fprintf("Closes attraction point\n");
            end
        else
            fprintf("No target\n");
            self.target = [];
        end



        if map.get_size() > 2
            map_points = zeros(2, map.get_size());
            for i = 1:map.get_size()
                map_points(:,i) = map.get_state_i(i);
            end
            pts_x = map_points(1,:);
            pts_y = map_points(2,:);

            DT = delaunayTriangulation(map_points');
            F = freeBoundary(DT);
            hole = polyshape(pts_x(F(:,1)), pts_y(F(:,1)));
            hole = polybuffer(hole, 0.1);
            if ~isempty(self.allowed_region)
                region = self.allowed_region; % polyshape(self.allowed_region(:,1), self.allowed_region(:,2));
                self.allowed_region = subtract(region, hole);
            end
        end

        if all(size(self.prev_target) == size(self.target)) && ~isempty(self.target) && all(self.prev_target == self.target)
            self.target_count = self.target_count + 1;
            if self.target_count > 10
                self.wrong_state_pos = self.target;
            end
        else
            self.target_count = 0;
        end

    end % plan_motion function


    function [dir, d_min] = build_repulsive_direction(self, manipulator)

        xEE = manipulator.get_EE_state(true);
        xEE = xEE(1:2);

        dir = [0; 0];
        d_min = Inf;

        for i = 1:size(self.repulsion_points, 2)
            delta = xEE - self.repulsion_points(:,i);
            d = norm(delta);
            if d < d_min
                d_min = d;
            end
            delta = delta / d;
            dir = dir + delta;
        end

    end % build_repulsive_direction function


    function w = weight(self, distance)
        w = pi/2 - atan(distance*5);
    end


end % methods

end % Object class
