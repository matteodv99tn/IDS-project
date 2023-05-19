classdef MotionPlanner < handle

properties %% ---- Attributes of the class --------------------------------------------------------
    home_config = pi/2 * ones(3, 1); 
    idx_to_follow = 0;
    robot     = Manipulator(5,5);        % initialize the robot and the camera     
    cam     = Camera(60*pi/180, 201);
   
end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = MotionPlanner() 
    end % MotionPlanner constructor

    function plan_motion(self, manipulator, map)
        
        config = get_current_configuration();
        x_EE = manipulator.get_EE_state();

        %Check for the current point
        if self.idx_to_follow ~= 0
            eps = map.get_max_uncertainty(self.idx_to_follow);         %checking for high uncertainity
             if eps < config.planner.search_th
                 self.idx_to_follow = 0;
             end
        end

        %Selecting new point
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


        end

        % Check which point to follow 
        % TODO

        % Compute the velocity vector 
        vel = zeros(3, 1);
        % TODO
        x_target = map.get_state_i(self.idx_to_follow)
        direction = x_target - x_EE(1:2);
        alpha  = atan2(direction(2),direction(1)); 
        gamma = x_EE(3) - alpha;
        omega = -gamma*0.5;
        normalizedVector = direction/norm(direction);
        des_vel = 0.5;
       


        v_EE = manipulator.get_EE_velocity();
        dist = norm(direction);
        v_radial = config.planner.kp_radial*(dist-config.planner.r_target);
        if v_radial > config.planner.v_des
            v_radial = config.planner.v_des;
        end
        v_tangent = sqrt(config.planner.v_des^2 - v_radial^2);
        tan_dir = [direction(2);direction(1)];
        project = v_EE(1:2)'*tan_dir;
        tan_dir = (tan_dir*project)/abs(project);

         vel = [v_radial*normalizedVector + v_tangent*tan_dir; omega];

       
        manipulator.controller.set_target(vel);
        
    end % plan_motion function

    
end % methods

end % Object class
