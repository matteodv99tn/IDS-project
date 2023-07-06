classdef Camera < handle

properties %% ---- Attributes of the class --------------------------------------------------------

    n_points;                       % number of points collected in the field of view
    fov;                            % field of view (in radians)
    angles;                         % angles that needs to be tested
    polar_covariance;               % covariance matrix of the uncertainty of polar measurements

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = Camera(fov, n_points)
        % If no argument are provided, the camera will generate assuming the default values.
        % The first input is the field of view, while the second is the number of points acquired 
        % at each scan.

        config = get_current_configuration();

        self.fov        = config.camera.defaults.fov;
        self.n_points   = config.camera.defaults.n_points;

        if nargin == 1
            self.n_points = n_points;
        elseif nargin == 2
            self.n_points = n_points;
            self.fov      = fov;
        end

        self.angles     = linspace(-self.fov/2, self.fov/2, self.n_points);
        self.polar_covariance = diag([config.camera.d_std, config.camera.theta_std]);

    end % Camera constructor


    function plot(self, RF)
        % In order to properly plot the camera, it's reference frame should be provided.
        % If this doesn't apply, it's implicitly assumed to be in the zero coordinate.
        if nargin == 1
            RF = eye(3);
        end
        OO      = [0; 0; 1];
        r       = 5;
        v       = [r*cos(self.angles); r*sin(self.angles); ones(1, self.n_points)];
        data    = RF * [OO, v];
        plt     = fill(data(1,:), data(2,:), "cyan");
        plt.EdgeAlpha = 0;
        plt.FaceAlpha = 0.3;
    end % plot function 


end % methods

end % Camera class
