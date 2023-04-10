classdef BaseController < handle

properties %% ---- Attributes of the class --------------------------------------------------------

    dt;
    target;

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = BaseController(fov, n_points)

        config = get_current_configuration();

        self.dt = config.simulation.dt;
        self.target = zeros(3, 1);
    end % BaseController constructor


    function tau = compute_tau(self, manipulator)
        tau = zeros(3, 1);
    end


    function set_target(self, target)
        self.target = target;
    end



end % methods

end % BaseController class
