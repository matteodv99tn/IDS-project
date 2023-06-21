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

        dir = dir/norm(dir) * 0.5;

        alpha = atan2(dir(2), dir(1));
        gamma = mod(xEE(3) - alpha + pi, 2*pi) - pi;
        omega = -gamma*2;

        safety_factor = 10;
        x_EE_next = xEE(1:2) + safety_factor*dir*manipulator.dt;
        if ~inpolygon(x_EE_next(1), x_EE_next(2), self.allowed_region(:,1), self.allowed_region(:,2))
            dir = [0; 0];
        end

        manipulator.controller.set_target([dir; omega]);
    end

    function plan_motion(self, manipulator, map)

        config = get_current_configuration();
        x_EE = manipulator.get_EE_state(true);

        feas_points = [];
        for i = 1:map.get_size()
            pt = map.get_state_i(i);
            if inpolygon(pt(1), pt(2), self.allowed_region(:,1), self.allowed_region(:,2))
                feas_points = [feas_points, i];
            end
        end

        if isempty(feas_points) && map.get_size() == 0
            self.target = [];
            return
        end

        if isempty(feas_points) && map.get_size() ~= 0
            self.target = map.centroid();
            return;
        end


        % Order the points
        feas_states = zeros(2, length(feas_points));
        for i = 1:length(feas_points)
            feas_states(:,i) = map.get_state_i(feas_points(i));
        end
        delta = feas_states - x_EE(1:2);
        dists = sqrt(sum(delta.^2, 1));
        [~, idx] = sort(dists);
        feas_points = feas_points(idx);

        eps = 0;
        i = 0;
        while eps < config.planner.search_th
            i = i + 1;
            eps = map.get_max_uncertainty(feas_points(i));
            target = map.get_state_i(feas_points(i));
        end

        self.target = target;
    end % plan_motion function


end % methods

end % Object class
