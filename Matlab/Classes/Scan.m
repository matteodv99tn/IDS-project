classdef Scan < handle

properties %% ---- Attributes of the class --------------------------------------------------------

    cartesian_points;               % 3xN matrix containing the measured points in cartesian world
    points_uncertainty;             % Nx2x2 matrix containing the (x, y) covariance matrix of each
                                    % point 

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = Scan(manipulator, camera, object)
    end % Scan constructor


    function plot(self)
    end % plot function 

    
    function [features, feat_covariance] = extract_features(self)
    end


end % methods

end % Scan class
