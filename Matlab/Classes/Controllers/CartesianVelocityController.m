classdef CartesianVelocityController < BaseController

properties %% ---- Attributes of the class --------------------------------------------------------


    Kp = 10*eye(3);
    Ki = 1*eye(3);
    acc_err = zeros(3,1);
    clip_th = [0.3; 0.3; 0.1];

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = CartesianVelocityController()
    end % CartesianVelocityController constructor


    function tau = compute_tau(self, manipulator)
        
        compute_tau@BaseController(self);

        vel_meas        = manipulator.get_EE_velocity();
        vel_ref         = self.target;
        err             = vel_ref - vel_meas;
        self.acc_err    = self.acc_err + err*self.dt;
        ddx             = self.Kp*err + self.Ki*self.acc_err;

        self.clip_integral_error();

        J       = manipulator.EE_jacobian();
        M       = manipulator.mass_matrix();
        h       = manipulator.bias_forces();
        Lambda  = (J * M^(-1) * transpose(J))^(-1);
        mu      = Lambda * J * M^(-1) * h;
        tau     = transpose(J) * (Lambda*ddx + mu);
    end


    function clip_integral_error(self)
        for i = 1:3
            if abs(self.acc_err(i)) > self.clip_th(i)
                self.acc_err(i) = sign(self.acc_err(i))*self.clip_th(i);
            end
        end
    end



end % methods

end % CartesianVelocityController class
