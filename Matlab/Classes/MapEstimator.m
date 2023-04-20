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

        [seeds, features, n_removed] = extract_features(scan);
        correspondences = self.find_correspondences(manipulator, features);

        x = [manipulator.q; self.x]
        H = zeros(2*size(correspondences, 2), length(x));
        z = zeros(2*size(correspondences, 2), 1);

        for i = 1:size(correspondences, 2)
            h, H_q, H_o = inverse_observation_model( ...
                                        manipulator, ...
                                        features(:, correspondences(1, i)) ...
                                        );
            H(2*i-1:2*i, 1:3) = H_q;
            H(2*i-1:2*i, 3 + correspondences(2:3, i)) = H_o;
            z(2*i-1:2*i) = h;
        end


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

end % methods

end % MapEstimator class
