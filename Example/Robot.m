%{
    Robot class 

    The state vector is:
        (x, y, theta)
    The input is just the linear velocity (along x axis) and the angular vel.
        (v, w)
    The kinematic model used is the one of the unicycle, i.e.
        x_dot = v * cos(theta)
        y_dot = v * sin(theta)
        theta_dot = w
    that discretized is
        x_k+1 = x_k + v_k * cos(theta_k) * dt
        y_k+1 = y_k + v_k * sin(theta_k) * dt
        theta_k+1 = theta_k + w_k * dt

    For what concerns the sensing we have:
        - a odometry that evaluates the linear and angular velocity;
        - a gps that allows to measure the position of the robot in the 
          inertial frame;
        - a system that tracks the object of interest in a polar way measured 
          w.r.t. the robot local frame.

%}
classdef Robot < handle

properties 
    k;              % current time step
    dt;             % discretization time-step

    x;              % actual_state vector
    x_hat;          % estimated state vector
    P_hat;          % covariance matrix of the state estimate
    Sigma;          % uncertainty history
    X;              % actual state history
    X_hat;          % estimated state history
    U;              % applied input history 
    U_meas;         % measured input history
    R_odo;          % covariance matrix of the odometry (for prediction step)
    R_gps;          % covariance matrix of the gps (for update step)   
    R_meas;         % covariance matrix of the object measurement
    R_mag;

    C;
    Z;
    H;

    meas_z;
    meas_cov;
    

end % properties

methods

    function self = Robot(time_len, dt)
        self.k      = 0;
        self.dt     = dt;
        self.x      = zeros(3, 1); 
        self.x_hat  = zeros(3, 1);
        self.P_hat  = zeros(3, 3);
        self.Sigma  = zeros(3, time_len);
        self.X      = zeros(3, time_len);
        self.X_hat  = zeros(3, time_len);
        self.R_odo  = zeros(2, 2);
        self.R_gps  = zeros(2, 2);
        self.R_meas = 0 * diag([0.1, 0.001]);
        self.R_mag  = 1.4 * pi / 180;

        self.Z = [];
        self.C = [];
        self.H = [];
        self.meas_z = zeros(2, time_len);
        self.meas_cov = zeros(2, 2, time_len);

    end % constructor
    

    function plot(self) 

        [x_true, y_true] = draw_triangle(1, self.x);
        [x_est, y_est] = draw_triangle(1, self.x_hat);
        [x_unc, y_unc] = uncertainty_ellipsoid(self.x_hat(1:2), self.P_hat(1:2, 1:2));

        plot(x_true, y_true, "k--");
        plot(self.x(1), self.x(2), "ko");
        plot(x_est, y_est, "b");
        plot(x_unc, y_unc, "b--");
        
    end


    function step_time(self, t)
        if nargin == 2
            self.k = t;
        else
            self.k = self.k + 1;
        end
    end


    function KF_prediction_step(self, u)
        % Prediction step of the extended Kalman filter 

        % Compute the input and store them
        U_meas(:, self.k)   = u;
        u_meas              = u;
        u_applied           = u + mvnrnd(zeros(2,1), self.R_odo)'; 
        U(:, self.k)        = u_applied;

        % Update the kinematic model (both true and estimated)
        self.x      = unicycle_kinematics(self.x, u_applied, self.dt);
        self.x_hat  = unicycle_kinematics(self.x_hat, u_meas, self.dt);

        % Update the covariance matrix of the state estimate
        A = [ 1, 0, -u_applied(1)*sin(self.x_hat(3))*self.dt; ...
              0, 1,  u_applied(1)*cos(self.x_hat(3))*self.dt; ...
              0, 0,  1];
        G = [ cos(self.x_hat(3))*self.dt, 0;
              sin(self.x_hat(3))*self.dt, 0;
              0,                          1];

        self.P_hat          = A*self.P_hat*A' + G*self.R_odo*G';
        self.X(:, self.k)   = self.x;
    end % KF_prediction_step function


    function KF_update_step(self)
        % Update step of the extended Kalman filter

        R = blkdiag(self.R_gps, self.R_mag);
        z = [ ...
            self.x(1:2) + mvnrnd(zeros(2,1), self.R_gps)'; ...
            self.x(3) + randn(1)*self.R_mag ...
            ];
        H = eye(3);
        P = self.P_hat;
        x = self.x_hat;

        S = H*P*H' + R;
        W = P*H'*inv(S);
        x = x + W*(z - x);
        P = (eye(3) - W*H)*P;

        self.x_hat(3) = wrap_to_pi(self.x_hat(3));

        self.x_hat              = x;
        self.X_hat(:, self.k)   = x;
        self.P_hat              = P;
        self.Sigma(:, self.k)   = sqrt(diag(P));
    end % KF_update_step function


    function z = build_measure(self, point)
        % Given a point in the global frame, the function builds the associated
        % robot measurement (already with noise)

        M01 = [ cos(self.x(3)), -sin(self.x(3)), self.x(1);
                sin(self.x(3)),  cos(self.x(3)), self.x(2);
                0,               0,              1];
        M10 = inv(M01);
        p   = M10*[point; 1];
        z   = [ sqrt(p(1)^2 + p(2)^2);
                atan2(p(2), p(1))];
        z   = z + mvnrnd(zeros(2,1), self.R_meas)';
    end % build_measure function

    function H = observation_jacobian(self, z)
        H           = zeros(2, 5);

        xrob        = self.x_hat(1);
        yrob        = self.x_hat(2);
        theta       = self.x_hat(3);
        rho         = z(1);
        phi         = z(2);
        delta       = theta - phi;
        cosTheta    = cos(theta);
        sinTheta    = sin(theta);
        cosDelta    = cos(delta);
        sinDelta    = sin(delta);
        
        H(1, 1)     = -cosTheta;
        H(1, 2)     = -sinTheta;
        H(1, 3)     = -yrob*cosTheta + xrob*sinTheta - rho*sinDelta;
        H(1, 4)     = cosDelta;
        H(1, 5)     = rho * sinDelta;
        H(2, 1)     = sinTheta;
        H(2, 2)     = -cosTheta;
        H(2, 3)     = xrob*cosTheta + yrob*sinTheta - rho*cosDelta;
        H(2, 4)     = -sinDelta;
        H(2, 5)     = rho * cosDelta;
    end % observation_jacobian function

    
    function [z, R] = point_estimate(self, point)
        % Given a point in the map, the function computes a measurement of it.
        % Then it performs another projection based on it's current estimate to
        % the point's position in the map. In this case the combined 
        % uncertainty covariance matrix is computed.
        z_pt  = self.build_measure(point);
        xrob  = self.x_hat(1);
        yrob  = self.x_hat(2);
        theta = self.x_hat(3);
        rho   = z_pt(1);
        phi   = z_pt(2);

        p     = [rho * cos(phi); rho * sin(phi); 1];
        M01   = [ cos(theta), -sin(theta), xrob; ...
                  sin(theta),  cos(theta), yrob; ...
                  0,           0,          1];
        Q     = blkdiag(self.P_hat, self.R_meas);
        H     = self.observation_jacobian(z_pt);

        z     = M01 * p;
        z     = z(1:2);
        R     = H*Q*H';
    end % point_estimate function



    function [F, a] = build_composite_informations(self, point)
        % Given a point in the global frame, this function builds the composite 
        % information matrix F and state a for the current robot.
        % This will be used later for the distributed weighted least square.

        [z, C] = self.point_estimate(point);
        H      = eye(2);
        Cinv   = inv(C);
        self.C = blkdiag(self.C, C);
        self.Z = [self.Z; z];
        self.H = [self.H; H];
        F      = H' * Cinv * H;
        a      = H' * Cinv * z;

        self.meas_z(:, self.k) = z;
        self.meas_cov(:, :, self.k) = C;
    end


    function set_gps_uncertainty(self, R)
        self.R_gps = R;
    end % set_gps_uncertainty function

end % methods
end % Robot class


function x_new = unicycle_kinematics(x, u, dt)
    A       = eye(3);
    B       = [ cos(x(3)), 0;
                sin(x(3)), 0;
                0,         1];
    x_new   = A*x + B*u*dt;
end

function [x, y] = draw_triangle(size, ref_frame)
    pts = size * [... 
        1, -1, -1, 1; ...
        0, -0.5, 0.5, 0; ...
        1, 1, 1, 1 ...
        ];
    M = [cos(ref_frame(3)), -sin(ref_frame(3)), ref_frame(1); ...
         sin(ref_frame(3)),  cos(ref_frame(3)), ref_frame(2); ...
         0,                  0,                  1];
    pts = M * pts;
    x = pts(1, :);
    y = pts(2, :);
end
