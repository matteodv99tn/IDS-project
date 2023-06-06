classdef Robot < handle

    %  ____                            _   _           
    % |  _ \ _ __ ___  _ __   ___ _ __| |_(_) ___  ___ 
    % | |_) | '__/ _ \| '_ \ / _ \ '__| __| |/ _ \/ __|
    % |  __/| | | (_) | |_) |  __/ |  | |_| |  __/\__ \
    % |_|   |_|  \___/| .__/ \___|_|   \__|_|\___||___/
    %                 |_|                              
    properties

        x_robot;        % state of the robot
        R_meas;         % covariance matrix of the generated measurement

        x_est;          % local estimate of the points
        P_est;          % local covariance of the points estimate


    end % properties



    %  __  __      _   _               _     
    % |  \/  | ___| |_| |__   ___   __| |___ 
    % | |\/| |/ _ \ __| '_ \ / _ \ / _` / __|
    % | |  | |  __/ |_| | | | (_) | (_| \__ \
    % |_|  |_|\___|\__|_| |_|\___/ \__,_|___/
    methods

        function self = Robot(N_pts)
            % N_pts: number of points to be estimated
            x           = zeros(3, 1);
            x(1)        = -10 + 20*rand(1);
            x(2)        = -10 + 20*rand(1);
            x(3)        = 2*pi*rand(1);
            self.x_robot = x;
            
            tmp         = rand(2, 2);
            self.R_meas = tmp * tmp';

            dim_est     = 2*N_pts;
            self.x_est  = zeros(dim_est, 1);
            self.P_est  = 20*eye(dim_est);
        end % constructor

        
        function plot(self, z)
            M01 = self.RF();
            p1  = [1, -0.5, -0.5, 1; ...
                   0, -0.5,  0.5, 0; ...
                   1,  1,    1,   1];
            p0 = M01 * p1;
            x  = p0(1, :);
            y  = p0(2, :);
            plot(x, y, "k");
            plot(self.x_robot(1), self.x_robot(2), "ko");

            if nargin > 1
                [x, y] = uncertainty_ellipsoid(z, self.R_meas);
                plot(x, y, "--");
            end
        end % plot function
        
        
        %  ____        _     _ _      
        % |  _ \ _   _| |__ | (_) ___ 
        % | |_) | | | | '_ \| | |/ __|
        % |  __/| |_| | |_) | | | (__ 
        % |_|    \__,_|_.__/|_|_|\___|
        %                             
        function move(self)
            dt      = 0.05;
            theta   = self.x_robot(3);
            v       = [cos(theta); sin(theta); 0];
            omega   = [0; 0; 1];
            u1      = 1 + randn(1)/3;
            u2      = 0.4 + randn(1)/4;
            self.x_robot = self.x_robot + u1*v*dt + u2*omega*dt;
        end % move function


        function [z, R] = get_measurement(self, pt)
            dir = self.x_robot(1:2) - pt;
            dir = dir / norm(dir);
            P   = [dir, [dir(2); -dir(1)]];
            M   = diag([0.1, 2]);
            self.R_meas = P*M*P';
            z = pt + mvnrnd(zeros(2, 1), self.R_meas)';
            if nargout == 2
                R = self.R_meas;
            end
        end % get_measurement function

        
        function [F, a] = build_composite_informations(self, z)
            H       = eye(1);
            R       = self.R_meas;
            Rinv    = inv(R);
            F       = H' * Rinv * H;
            a       = H' * Rinv * z;
        end % build_composite_informations function


        function distributed_KF_update(self, F, a, N_robots)
            P          = inv(self.P_est + N_robots*F);
            self.x_est = P * (self.P_est*self.x_est + N_robots*a);
            self.P_est = P;
        end


        %  ____       _            _       
        % |  _ \ _ __(_)_   ____ _| |_ ___ 
        % | |_) | '__| \ \ / / _` | __/ _ \
        % |  __/| |  | |\ V / (_| | ||  __/
        % |_|   |_|  |_| \_/ \__,_|\__\___|
        %                                  
        function M = RF(self)
            x = self.x_robot(1);
            y = self.x_robot(2);
            theta = self.x_robot(3);
            M = [cos(theta),    -sin(theta),    x; ...
                 sin(theta),     cos(theta),    y; ...
                 0,              0,             1];
        end % RF function



    end % methods


end
