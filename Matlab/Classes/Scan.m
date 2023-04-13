classdef Scan < handle

properties %% ---- Attributes of the class --------------------------------------------------------

    polar_measures;                 % 2xN matrix containing the measurements in polar coordinate
                                    % (dist, angle)
    cartesian_points;               % 3xN matrix containing the measured points in cartesian world
    points_uncertainty;             % Nx2x2 matrix containing the (x, y) covariance matrix of each
                                    % point 

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = Scan(manipulator, camera, object)

        config = get_current_configuration();
        d_max  = config.camera.d_max;
        
        % Here I assume that if the manipulator argument is not an actual manipulator, then it's 
        % a reference frame where the camera is placed
        if class(manipulator) == "Manipulator"
            RF_camera = manipulator.EE_frame();
        else
            RF_camera = manipulator;
        end
        polygon   = object.get_projected_polygon();

        angles              = camera.angles + randn(1, camera.n_points)*camera.polar_covariance(2, 2);
        polar_directions    = RF_camera * [cos(angles); sin(angles); zeros(1, camera.n_points)];
        camera_origin       = RF_camera * [0; 0; 1];

        polar_meas          = d_max * ones(1, camera.n_points);
        curr_meas           = zeros(1, camera.n_points);

        for i = 1:size(polygon, 2)-1
            P1 = polygon(1:2, i);
            P2 = polygon(1:2, i+1);

            for j = 1:camera.n_points 
                v = polar_directions(1:2, j);
                curr_meas(j) = cast_ray(P1, P2, camera_origin(1:2), v);
            end
            
            correct_meas = curr_meas < polar_meas;
            polar_meas(correct_meas) = curr_meas(correct_meas);
        end


        noise       = randn(1, camera.n_points) * camera.polar_covariance(1, 1);
        polar_meas = polar_meas + noise .* (polar_meas < d_max);        

        self.cartesian_points = [ ...
                polar_meas .* cos(camera.angles); ...
                polar_meas .* sin(camera.angles); ...
                ones(1, camera.n_points) ...
                ];
        self.polar_measures = [ ...
                polar_meas; ...
                camera.angles; ...
                ];

    end % Scan constructor


    function plot(self)
        plot(self.cartesian_points(1,:), self.cartesian_points(2,:), ".k");
    end % plot function 

    
    % function [features, feat_covariance] = extract_features(self)
    % end


end % methods

end % Scan class
