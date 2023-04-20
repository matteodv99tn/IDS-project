classdef MapEstimator

properties %% ---- Attributes of the class --------------------------------------------------------
    
    x;                          % state of the map 
    P;                          % covariance matrix of the map

    buffer;

end % properties

methods %% ---- Member functions ------------------------------------------------------------------

    function self = MapEstimator()

        self.x = [];
        self.P = [];

        self.buffer = {};
        
    end % MapEstimator constructor


    function plot(self)
    end % plot function 


    function KF_update_step(self, manipulator, scan, camera)

        config = get_current_configuration();

        [seeds, features, n_removed]    = extract_features(scan);
        correspondences                 = self.find_correspondences(manipulator, features);
        n_correspondences               = size(correspondences, 2);


        if n_correspondences > 0
            dim_x = length(self.x);
            dim_z = 3 + 2*n_correspondences;
            x = self.x; 
            P = self.P;
            z = zeros(dim_z, 1);
            H = zeros(dim_z, dim_x);
            R = zeros(dim_z, dim_z);
            
            R(1:3, 1:3) = manipulator.R_q;
            for i = 1:size(correspondences, 2)
                x_corr = correspondences(2:3, i);
                zi = features(:, correspondences(1, i));
                Ri = cartesian_covariance_from_polar( ...
                                            zi, ...
                                            camera.polar_covariance ...
                                            );
                z(2+2*i:3+2*i) = zi;
                R(2+2*i:3+2*i, 2+2*i:3+2*i) = Ri;
                h, H_q, H_o = direct_observation_model( ...
                                            manipulator, ...
                                            x(x_corr) ...
                                            );
                H(1:3, x_corr) = H_q;
                H(2+2*i:3+2*i, x_corr) = H_o;
            end

            S = H*P*transpose(H) + R;
            W = P*transpose(H)*inv(S);
            x = x + W*(z - h);
            P = (eye(dim_x) - W*H)*P;

            self.x = x;
            self.P = P;
        end % update step 
        


    end % KF_update_step function

    
    function correspondences = find_correspondences(self, manipulator, camera, features)
        % This function aims at recognizing if the extracted features have already been seen in the
        % the current map.
        % For each extracted feature, we compute the inverse observation to get the corresponding
        % expected landmark. We then use the Mahalanobis distance to check if there are some valid
        % correspondences.
        % The output matrix is a 3xN matrix for which:
        %  - the first row contains the index of the feature;
        %  - the second and third rows contain the indices of the corresponding landmark in the 
        %    state vector.
        
        correspondences = [];
        
        for i = 1:size(features, 2)
            h, H_q, H_o = inverse_observation_model(manipulator, features(:, i));
            H = [H_q, H_o];
            P = blkdiag(manipulator.R_q, camera.polar_covariance);
            S = H * P * transpose(H);

            if j = 1:length(self.x)/2
                if mahalanobis_distance(self.x(2*j-1:2*j), h, S) < config.estimator.mahalanobis_th 
                    correspondences(:, end+1) = [i; 2*j-1; 2*j];
                end
            end
        end
    end % find_correspondences function


    function add_state(self, manipulator, feature)
    end

end % methods

end % MapEstimator class
