classdef MotionPlanner < handle

properties %% ---- Attributes of the class --------------------------------------------------------
    home_config = pi/2 * ones(3, 1); 
    idx_to_follow = 0;
end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = MotionPlanner() 
         
    end % MotionPlanner constructor

    function plan_motion(self, manipulator, map)

        % Check which point to follow 
        % TODO

        % Compute the velocity vector 
        vel = zeros(3, 1);
        % TODO

        
        manipulator.controller.set_target(vel);
        
    end % plan_motion function

end % methods

end % Object class
