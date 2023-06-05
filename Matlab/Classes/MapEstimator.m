classdef MapEstimator < handle

properties %% ---- Attributes of the class --------------------------------------------------------
    
    x;                          % state of the map 
    P;                          % covariance matrix of the map
    z;
    H;
    R;


    new_observations;
    x_new;
    P_new;

    F;
    a;

    buffer;

end % properties

methods %% ---- Member functions ------------------------------------------------------------------

    function self = MapEstimator(stop_exec)

        self.x = [];
        self.P = [];
        if nargin < 1
            self.new_observations = MapEstimator(true);
        end

        self.buffer = {};
        
    end % MapEstimator constructor

    function plot(self)
        for i = 1:self.get_size()
            mu = self.get_state_i(i);
            sigma = self.get_covariance_i(i);
            [x, y] = uncertainty_ellipsoid(mu, sigma);
            plot(x, y, "b");
        end
        x = self.z(1:2:end);
        y = self.z(2:2:end);
        plot(x, y, "*r");
    end % plot function 

    function xi = get_state_i(self, idx) 
        xi = self.x(2*idx-1:2*idx);
    end % get_state_i function 

    function Pi = get_covariance_i(self, idx)
        Pi = self.P(2*idx-1:2*idx, 2*idx-1:2*idx);
    end % get_covariance_i function


    function n = get_size(self) 
        n = length(self.x) / 2;
    end % get_map_size function

    
    function eps = get_max_uncertainty(self, idx)
        % Returns the maximum eigenvalue of the covariance matrix for the i-th
        % state
        eps = max(abs(eig(self.get_covariance_i(idx))));
    end
    
    function join(self, arg1, arg2)
        % Joins either 2 maps or a new state+covariance to the current map
        
        if nargin == 2 % We want to join 2 maps 
            other = arg1; 
            self.x = [self.x; other.x];
            self.P = blkdiag(self.P, other.P);
        end 
        if nargin == 3 % we want to join a state (with it's covariance)
            x = arg1;
            P = arg2;
            self.x = [self.x; x];
            self.P = blkdiag(self.P, P);
        end
    end


    function conditional_join(self, other)
        %% Join two maps checking for overalapping landmarks 
        config = get_current_configuration();
        
        for i = 1:other.get_size() 
            x_other = other.get_state_i(i);
            P_other = other.get_covariance_i(i);
            can_join = true;

            for j = 1:self.get_size() 
                x_self = self.get_state_i(j);
                P_self = self.get_covariance_i(j);
                
                mh_dist = mahalanobis_distance(x_self, x_other, P_self);
                if mh_dist < config.estimator.mahalanobis_th 
                    %% Here perform the WLS
                    H = [eye(2); eye(2)];
                    z = [x_self; x_other];
                    R = blkdiag(P_self, P_other);
                    Rinv = inv(R);

                    P_self = inv(H' * Rinv * H);
                    x_self = P_self * H' * Rinv * z;

                    self.x(2*j-1:2*j) = x_self;
                    self.P(2*j-1:2*j, 2*j-1:2*j) = P_self;
                    can_join = false;
                    break;
                end
            end

            if can_join
                self.join(x_other, P_other);
            end
        end
    end

    function process_scan(self, manipulator, scan, camera) 

        %% Process laserscan and generate correspondences
        config = get_current_configuration();
        [seeds, features, n_removed]    = extract_features(scan);
        correspondences                 = self.find_correspondences(manipulator, camera, features);
        n_correspondences               = 0;
        if ~isempty(correspondences)
            n_correspondences           = size(correspondences, 2);
        end

        dim_z = length(self.x); 
        dim_x = length(self.x);
        H = eye(dim_z);
        h = self.x;
        z = zeros(dim_z, 1);
        R = 10e6 * eye(dim_z);

        %% Precompute things for the distributed KF update
        if n_correspondences > 0
            [z, R] = project_features( ...
                            manipulator, ...
                            camera, ...
                            features(:, correspondences(1, :)) ...
                            );
            z_old = z;
            R_old = R;

            z = zeros(dim_z, 1);
            R = 10e6 * eye(dim_z);
            
            for i = 1:size(correspondences, 2)
                x_corr = correspondences(2:3, i);
                z(x_corr) = z_old(2*i-1:2*i);
                R(x_corr, x_corr) = R_old(2*i-1:2*i, 2*i-1:2*i);
            end

        end % update step 
        self.z = z; 
        self.H = H;
        self.R = R;
        self.build_composite_informations();

        %% Add unused information to buffer
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
    end

    
    function build_composite_informations(self) 
        H      = self.H;
        R      = self.R;
        z      = self.z;
        Rinv   = inv(R);
        F      = H' * Rinv * H;
        a      = H' * Rinv * z;
        self.F = F;
        self.a = a;
        if size(a, 2) > 1 
            warning("SIZE ERROR")
        end
    end


    function [F, a] = get_composite_informations(self);
        F = self.F;
        a = self.a;
    end


    function linear_consensus(self, F_other, a_other, q_self, q_other)
        self.F = self.F*q_self + F_other*q_other;
        self.a = self.a*q_self + a_other*q_other;
    end

    function KF_update_step(self, N)
        % N = Number of robots 
        if size(self.a, 2) > 1 
            warning("Invalid size");
        end
        P = inv(self.P + N*self.F);     
        self.x = P * (self.P*self.x + N*self.a); 
        self.P = P;
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
        self.new_observations = MapEstimator();
        self.x_new = [];
        self.P_new = [];
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
                self.new_observations.join(z_curr, R_curr);
                self.buffer{end}.z(:, idx_obs) = [];
                self.buffer{end}.R(:, :, idx_obs) = [];
                idx_obs = idx_obs - 1;
            end
            idx_obs = idx_obs + 1;
        end
    end % process_buffer function

end % methods

end % MapEstimator class
