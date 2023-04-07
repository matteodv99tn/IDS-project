classdef Scan < handle

properties %% ---- Attributes of the class --------------------------------------------------------

    cartesian_points;               % 3xN matrix containing the measured points in cartesian world
    points_uncertainty;             % Nx2x2 matrix containing the (x, y) covariance matrix of each
                                    % point 

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = Scan(manipulator, camera, object)

        global d_max;
        
        RF_camera = manipulator.EE_frame();
        polygon   = object.point_matrix;

        angles              = camera.angles + randn(1, camera.n_points)*camera.polar_covariance(2, 2);
        polar_directions    = RF_camera * [cos(angles); sin(angles); zeros(1, camera.n_points)];
        camera_origin       = RF.camera * [0; 0; 1];

        polar_meas = d_max * ones(1, camera.n_points);
        curr_meas = zeros(1, camera.n_points);

        for i = 1:size(polygon, 2)-1
            P1 = polygon(:, i);
            P2 = polygon(:, i+1);

            for j = 1:camera.n_points 
                curr_meas(j) = cast_ray(P1, P2, camera_origin(1:2), angles(j));
            end
            
            correct_meas = curr_meas < polar_meas;
            polar_meas(correct_meas) = curr_meas(correct_meas)
        end

        polar_meas = polar_meas * randn(1, camera.n_points)*camera.polar_covariance(1, 1);        

        self.cartesian_points = [ ...
                polar_meas .* cos(camera.angles); ...
                polar_meas .* sin(camera.angles); ...
                ones(1, camera.n_points) ...
                ];

    end % Scan constructor


    function plot(self)
        plot(self.cartesian_points[1,:], self.cartesian_points[2,:], ".k");
    end % plot function 

    
    function [features, feat_covariance] = extract_features(self)
    end


end % methods

end % Scan class
