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
    k = 0;

    allreg_hist;
    hist_k;

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = MotionPlanner()
        config = get_current_configuration();
        N = config.simulation.N_meas;
        self.allreg_hist = cell(1, N);
        self.hist_k  = 1;
    end % MotionPlanner constructor


    function save_region(self)
        self.allreg_hist{self.hist_k} = self.allowed_region;
        self.hist_k = self.hist_k + 1;
    end


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
            rad = norm(dir_target);
            dir_target = dir_target / rad;

            vel_ref = 0.3;

            v_rad = dir_target * (rad - 0.4);
            if norm(v_rad) > vel_ref
                v_rad = v_rad/norm(v_rad) * vel_ref;
                tan_vel = 0;
            else
                tan_vel = sqrt(vel_ref^2 - norm(v_rad)^2);
            end

            v_tan = [-dir_target(2); dir_target(1)] * tan_vel * cos(self.k/1000);
            self.k = self.k + 1;
            dir = v_rad + v_tan;

            incell = false;
            i = 0;
            while ~incell

                dir = dir/norm(dir) * vel_ref;
                xEE_next = xEE(1:2) + 2*dir*manipulator.dt;
                incell = isinterior(self.allowed_region, xEE_next(1), xEE_next(2));

                [dist, px, py] = p_poly_dist(xEE_next(1), xEE_next(2), self.allowed_region.Vertices(:,1), self.allowed_region.Vertices(:,2));
                delta = [px; py] - xEE_next;

                if ~incell
                    dir = dir + delta/norm(delta) * 1.1;
                end
                i = i + 1;
                if i > 100
                    fprintf("STUCK!\n");
                    dir = [0; 0];
                    incell = true;
                end
            end

            alpha = atan2(dir_target(2), dir_target(1));
            gamma = mod(xEE(3) - alpha + pi, 2*pi) - pi;
            omega = -gamma*4;
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
            else
                xEE = x_EE(1:2);
                dist = vecnorm(self.attraction_points - xEE);
                [~, idx] = min(dist);
                self.target = self.attraction_points(:,idx);
            end
        else
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
            hole = polybuffer(hole, 0.2);
            if ~isempty(self.allowed_region)
                region = self.allowed_region; % polyshape(self.allowed_region(:,1), self.allowed_region(:,2));
                self.allowed_region = subtract(region, hole);
            end
        end

        if all(size(self.prev_target) == size(self.target)) && ~isempty(self.target)
            tmp = self.prev_target == self.target;
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
