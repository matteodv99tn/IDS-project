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
        for i = 1:length(self.x) / 2
            mu = self.x(2*i-1:2*i);
            sigma = self.P(2*i-1:2*i, 2*i-1:2*i);
            [x, y] = uncertainty_ellipsoid(mu, sigma);
            plot(x, y, "b");
        end
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
            [z, R] = project_features( ...
                            manipulator, ...
                            camera, ...
                            features(:, correspondences(1, :)) ...
                            );
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
        
        % Add unused information to buffer
        new_obs = ones(1, size(features, 2)); 
        if ~isempty(correspondences)
            new_obs(correspondences(1, :)) = 0;
        end
        new_feat  = features(:, new_obs == 1);
        n_new_obs = sum(new_obs);
        self.buffer{end+1} = struct();
        self.buffer{end}.z = zeros(2, n_new_obs);
        self.buffer{end}.R = zeros(2, 2, n_new_obs);

        for i = 1:n_new_obs
            [z, R] = project_features( ...
                            manipulator, ...
                            camera, ...
                            new_feat(:, i) ...
                            );
            self.buffer{end}.z(:, i) = z;
            self.buffer{end}.R(:, :, i) = R;
        end

        if length(self.buffer) > config.estimator.buffer_size
            self.buffer(1) = [];
            self.process_buffer();
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

    
    function process_buffer(self)
        config = get_current_configuration();
        idx_obs = 1;
        while idx_obs  <= size(self.buffer{end}.z, 2)
            z_curr = self.buffer{end}.z(:, idx_obs);
            R_curr = self.buffer{end}.R(:, :, idx_obs);
            is_valid = false;

            for k = 1:length(self.buffer)-1
                if isempty(self.buffer{k})
                    is_valid = false;
                    break;
                end

                for i = 1:size(self.buffer{k}.z, 2)
                    z_cmp = self.buffer{k}.z(:, i);
                    if mahalanobis_distance(z_cmp, z_curr, R_curr) < config.estimator.mahalanobis_th
                        is_valid = true;
                        break;
                    end
                end
            end


            if is_valid
                self.x = [self.x; z_curr];
                self.P = blkdiag(self.P, R_curr);
                self.buffer{end}.z(:, idx_obs) = [];
                self.buffer{end}.R(:, :, idx_obs) = [];
                idx_obs = idx_obs - 1;
            end
            idx_obs = idx_obs + 1;
        end
    end % process_buffer function

end % methods

end % MapEstimator class
