classdef CartesianPointController < BaseController

properties %% ---- Attributes of the class --------------------------------------------------------


    Kp = 180*eye(3);
    Kd = 50*eye(3);

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = CartesianPointController()
    end % CartesianPointController constructor


    function tau = compute_tau(self, manipulator)
        x_pos = manipulator.get_EE_state();
        x_vel = manipulator.get_EE_velocity();
        x_ref = self.target;
        
        ddx   = -self.Kd*x_vel + self.Kp*(x_ref - x_pos);

        J       = manipulator.EE_jacobian();
        M       = manipulator.mass_matrix();
        h       = manipulator.bias_forces();
        Lambda  = (J * M^(-1) * transpose(J))^(-1);
        mu      = Lambda * J * M^(-1) * h;
        tau     = transpose(J) * (Lambda*ddx + mu);
    end



end % methods

end % CartesianPointController class
