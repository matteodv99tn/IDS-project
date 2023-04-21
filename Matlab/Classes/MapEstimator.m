classdef MapEstimator < handle

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
        correspondences                 = self.find_correspondences(manipulator, camera, features);
        n_correspondences               = 0;
        if ~isempty(correspondences)
            n_correspondences           = size(correspondences, 2);
        end


        if n_correspondences > 0
            [z, R] = project_features(manipulator, camera, features(:, correspondences(1, :)));
            dim_z = length(z);
            dim_x = length(self.x);
            x = self.x; 
            P = self.P;
            H = zeros(dim_z, dim_x);
            h = zeros(dim_z, 1);
            
            for i = 1:size(correspondences, 2)
                x_corr = correspondences(2:3, i);
                h(2*i-1:2*i) = x(x_corr); 
                H(2*i-1:2*i, x_corr) = eye(2);
            end

            S = H*P*transpose(H) + R;
            W = P*transpose(H)*inv(S);
            x = x + W*(z - h);
            P = (eye(dim_x) - W*H)*P;

            self.x = x;
            self.P = P;
        end % update step 
    
        if ~isempty(features)
            oo = ones(1, size(features, 2));
            if ~isempty(correspondences)
                oo(correspondences(1, :)) = 0;
            end
            tba = features(:, oo == 1);
            for i = 1:size(tba, 2)
                self.add_state(manipulator, camera, tba(:, i));
            end
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
        
        config = get_current_configuration();
        correspondences = [];
        
        for i = 1:size(features, 2)
            [h, S] = project_features(manipulator, camera, features(:, i));
            for j = 1:length(self.x)/2
                if mahalanobis_distance(self.x(2*j-1:2*j), h, S) < config.estimator.mahalanobis_th 
                    correspondences(:, end+1) = [i; 2*j-1; 2*j];
                end
            end
        end
    end % find_correspondences function


    function add_state(self, manipulator, camera, feature)
        [g, G_q, G_o] = inverse_observation_model(manipulator, feature);
        R = blkdiag(manipulator.R_q, camera.polar_covariance);
        G = [G_q, G_o];
        S = G * R * transpose(G);

        self.x = [self.x; g];
        self.P = blkdiag(self.P, S);
    end

end % methods

end % MapEstimator class
