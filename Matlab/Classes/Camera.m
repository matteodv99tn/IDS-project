classdef Camera < handle

properties %% ---- Attributes of the class --------------------------------------------------------

    n_points;                       % number of points collected in the field of view
    fov;                            % field of view (in radians)
    angles;                         % angles that needs to be tested
    polar_covariance;               % covariance matrix of the uncertainty of polar measurements

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = Camera(fov, n_points)

        config = get_current_configuration();

        self.n_points   = n_points;
        self.fov        = fov;
        self.angles     = linspace(-fov/2, fov/2, n_points);
        self.polar_covariance = diag([config.camera.d_std, config.camera.theta_std]);

    end % Camera constructor


    function plot(self)
    end % plot function 


end % methods

end % Camera class
