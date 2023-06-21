classdef MotionPlanner < handle

properties %% ---- Attributes of the class --------------------------------------------------------
    home_config = pi/2 * ones(3, 1);
    idx_to_follow = 0;
    robot     = Manipulator(5,5);        % initialize the robot and the camera
    cam     = Camera(60*pi/180, 201);

    allowed_region = [];
    target = [];

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = MotionPlanner()
    end % MotionPlanner constructor


    function update_target(self, manipulator)

        % If no target is set -> Rotate
        if isempty(self.target)
            manipulator.controller.set_target([0; 0; 2]);
            return;
        end

        xEE = manipulator.get_EE_state(true);
        dir = self.target - xEE(1:2);

        dir = dir/norm(dir) * 0.1;

        alpha = atan2(dir(2), dir(1));
        gamma = mod(xEE(3) - alpha + pi, 2*pi) - pi;
        omega = -gamma*2;

        manipulator.controller.set_target([dir; omega]);
    end

    function plan_motion(self, manipulator, map)

        config = get_current_configuration();
        x_EE = manipulator.get_EE_state(true);

        % Check for the current point
        if self.idx_to_follow ~= 0
            eps = map.get_max_uncertainty(self.idx_to_follow);   %checking for high uncertainity
            if eps < config.planner.search_th
                fprintf("Target reached!\n");
                self.idx_to_follow = 0;
            end
        end

        % Selecting new point
        if self.idx_to_follow == 0
            new_idx = 0;
            min_dist = 1e5;

            for i=1:map.get_size
                if map.get_max_uncertainty(i) > config.planner.search_th
                    dist = point_point_distance(map.get_state_i(i), x_EE(1:2));
                    if dist < min_dist
                        new_idx = i;
                        min_dist = dist;
                    end
                end
            end
            self.idx_to_follow = new_idx;
        end

        if self.idx_to_follow ~= 0
            self.target = map.get_state_i(self.idx_to_follow);
        else
            self.target = [];
        end
    end % plan_motion function


end % methods

end % Object class
