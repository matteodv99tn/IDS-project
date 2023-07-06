classdef CartesianPointController < BaseController

properties %% ---- Attributes of the class --------------------------------------------------------


    Kp = 1000*eye(3);
    Kd = 50*eye(3);

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = CartesianPointController()
    end % CartesianPointController constructor


    function tau = compute_tau(self, manipulator)
        
        compute_tau@BaseController(self);

        x_meas  = manipulator.get_EE_state(true);
        dx_meas = manipulator.get_EE_velocity(true);
        
        if self.k == 0
            x_des   = self.target;
            dx_des  = zeros(3, 1);
            ddx_des = zeros(3, 1);
        else
            if self.k == 1
                self.params = polynomial_interpolation(self.T, x_meas, dx_meas, self.target);
            end

            X = polynomial_evaluation(self.k*self.dt, self.params);
            x_des   = X(:, 1);
            dx_des  = X(:, 2);
            ddx_des = X(:, 3);
            self.k  = self.k + 1;
        end

        x_pos = manipulator.get_EE_state(true);
        x_vel = manipulator.get_EE_velocity(true);
        x_ref = self.target;
        
        ddx     = ddx_des + self.Kd*(dx_des-dx_meas) + self.Kp*(x_des-x_meas);

        J       = manipulator.EE_jacobian(true);
        M       = manipulator.mass_matrix(true);
        h       = manipulator.bias_forces(true);
        Lambda  = (J * M^(-1) * transpose(J))^(-1);
        mu      = Lambda * J * M^(-1) * h;
        tau     = transpose(J) * (Lambda*ddx + mu);
    end



end % methods

end % CartesianPointController class
