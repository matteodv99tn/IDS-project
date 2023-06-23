classdef CartesianVelocityController < BaseController

properties %% ---- Attributes of the class --------------------------------------------------------


    Kp = 1000*eye(3);
    Ki = 1*eye(3);
    acc_err = zeros(3,1);
    clip_th = 10*[0.3; 0.3; 0.1];

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = CartesianVelocityController()
    end % CartesianVelocityController constructor


    function tau = compute_tau(self, manipulator)

        compute_tau@BaseController(self);

        vel_meas        = manipulator.get_EE_velocity(true);
        vel_ref         = self.target;
        err             = vel_ref - vel_meas;
        self.acc_err    = self.acc_err + err*self.dt;
        ddx             = self.Kp*err + self.Ki*self.acc_err;

        self.clip_integral_error();

        J       = manipulator.EE_jacobian(true);
        M       = manipulator.mass_matrix(true);
        h       = manipulator.bias_forces(true);
        Lambda  = (J * M^(-1) * transpose(J))^(-1);
        mu      = Lambda * J * M^(-1) * h;
        tau     = transpose(J) * (Lambda*ddx + mu);
        tau     = tau + self.collision_avoider(manipulator);
        % tau = M * inv(J) * ddx + h;
    end


    function clip_integral_error(self)
        for i = 1:3
            if abs(self.acc_err(i)) > self.clip_th(i)
                self.acc_err(i) = sign(self.acc_err(i))*self.clip_th(i);
            end
        end
    end


    function tau = collision_avoider(self, manipulator);
        tau = zeros(3, 1);
        for i = 1:length(manipulator.other_points_pos)
            pts = manipulator.other_points_pos{i};
            L1  = manipulator.L1;
            L2  = manipulator.L2;
            q1 = manipulator.q_est(1);
            q2 = manipulator.q_est(2);
            OO = manipulator.origin;
            A = OO + L1 * [cos(q1); sin(q1)];
            B = A + L2 * [cos(q1+q2); sin(q1+q2)];

            [d, px, py] = p_poly_dist(A(1), A(2), pts(:,1), pts(:,2));
            v1 = A - [px; py];
            v2 = [-sin(q1), cos(q1)];
            tau(1) = tau(1) + 100 * dot(v1, v2) / (d^2);

            [d, px, py] = p_poly_dist(B(1), B(2), pts(:,1), pts(:,2));
            v1 = B - [px; py];
            v2 = [-sin(q1+q2), cos(q1+q2)];
            tau(2) = tau(2) + 100 * dot(v1, v2) / (d^2);
        end
    end



end % methods

end % CartesianVelocityController class
