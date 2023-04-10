classdef JointPointController < BaseController

properties %% ---- Attributes of the class --------------------------------------------------------


    Kp = 300*eye(3);
    Kd = 30*eye(3);

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = JointPointController()
    end % JointPointController constructor


    function tau = compute_tau(self, manipulator)
        qdd_des = -self.Kd*manipulator.dq + self.Kp * (self.target - manipulator.q);
        tau     = manipulator.mass_matrix()*qdd_des + manipulator.bias_forces();
    end



end % methods

end % JointPointController class
