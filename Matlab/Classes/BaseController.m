classdef BaseController < handle

properties %% ---- Attributes of the class --------------------------------------------------------

    dt;
    target;

    target_queue = [];
    queue_times = [];
    k = 0;
    params;
    T;

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = BaseController(fov, n_points)

        config = get_current_configuration();

        self.dt = config.simulation.dt;
        self.target = zeros(3, 1);
    end % BaseController constructor


    function tau = compute_tau(self, manipulator)
        if self.k * self.dt >= self.T 
            self.k = 0;
        end
        if ~isempty(self.queue_times) && (self.k == 0)
            self.k = 1;
            self.T = self.queue_times(1);
            self.target = self.target_queue(:, 1);
            self.queue_times(1) = [];
            self.target_queue(:, 1) = [];
        end
        tau = zeros(3, 1);
    end


    function set_target(self, target)
        self.k = 0;
        self.target = target;
    end


    function enqueue_target(self, target, time)
        self.target_queue(:, end+1) = target;
        self.k = 0;
        if nargin == 3
            self.queue_times(end+1) = time;
        else
            self.queue_times(end+1) = 4;
        end
    end



end % methods

end % BaseController class
