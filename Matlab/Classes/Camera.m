classdef Camera < handle

properties %% ---- Attributes of the class --------------------------------------------------------

    n_points;                       % number of points collected in the field of view
    fov;                            % field of view (in radians)
    polar_covariance;               % covariance matrix of the uncertainty of polar measurements

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = Camera(L1, L2, O)
    end % Camera constructor


    function plot(self)
    end % plot function 


end % methods

end % Camera class
