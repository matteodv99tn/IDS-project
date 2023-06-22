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

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = MotionPlanner()
    end % MotionPlanner constructor


    function update_target(self, manipulator,  map)

        if isempty(self.target)
            omega = 0.2;
            dir = self.build_repulsive_direction(manipulator);
            dir = dir/norm(dir) * 0.5;
            manipulator.controller.set_target([dir; omega]);
        else
            xEE = manipulator.get_EE_state(true);
            dir_target = self.target - xEE(1:2);
            dir_repulsive = self.build_repulsive_direction(manipulator);
            dir_repulsive = dir_repulsive/norm(dir_repulsive) * 0.1;

            incell = false;
            i = 0;
            while ~incell
                dir = dir_target + i*dir_repulsive;
                dir = dir/norm(dir) * 0.5;
                x_EE_next = xEE(1:2) + 10*dir*manipulator.dt;
                incell = isinterior(self.allowed_region, x_EE_next(1), x_EE_next(2));
                i = i + 1;
            end
            alpha = atan2(dir(2), dir(1));
            gamma = mod(xEE(3) - alpha + pi, 2*pi) - pi;
            omega = -gamma*2;
        end

        manipulator.controller.set_target([dir; omega]);
    end


    function plan_motion(self, manipulator, map)

        config = get_current_configuration();
        x_EE = manipulator.get_EE_state(true);


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

        feas_points = isinterior(self.allowed_region, ...
                                      self.attraction_points(1,:), ...
                                      self.attraction_points(2,:));

        feas_points = self.attraction_points(:, feas_points);
        if ~isempty(feas_points)
            self.target = feas_points(:,1);
        else
            self.target = [];
        end
    end % plan_motion function


    function dir = build_repulsive_direction(self, manipulator)

        xEE = manipulator.get_EE_state(true);
        xEE = xEE(1:2);

        dir = [0; 0];

        for i = 1:size(self.repulsion_points, 2)
            delta = xEE - self.repulsion_points(:,i);
            delta = delta / norm(delta);
            dir = dir + delta;
        end

    end % build_repulsive_direction function


end % methods

end % Object class
