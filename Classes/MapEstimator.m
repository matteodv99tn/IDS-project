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
    z_tmp;
    R_tmp;

    buffer;


    seeds;
    features;

    k;
    x_hist;
    P_hist;
    feat_hist;
    z_hist;
    R_hist;
end % properties

methods %% ---- Member functions ------------------------------------------------------------------

    function self = MapEstimator(stop_exec)

        config = get_current_configuration();

        self.x = [];
        self.P = [];
        if nargin < 1
            self.new_observations = MapEstimator(true);
        end

        self.buffer = {};

        self.z_tmp = [];
        self.R_tmp = [];

        N = config.simulation.N_meas;
        self.k = 1;
        self.x_hist = cell(1, N);
        self.P_hist = cell(1, N);
        self.feat_hist = cell(1, N);
        self.z_hist = cell(1, N);
        self.R_hist = cell(1, N);

    end % MapEstimator constructor

    function plot(self)
        for i = 1:self.get_size()
            mu = self.get_state_i(i);
            sigma = self.get_covariance_i(i);
            [x, y] = uncertainty_ellipsoid(mu, sigma);
            plot(x, y, "b");
        end

        for i = 1:(size(self.z)/2)
            z = self.z(2*i-1:2*i);
            if all(z == [0; 0])
                continue;
            end
            P = self.R(2*i-1:2*i, 2*i-1:2*i);
            [x, y] = uncertainty_ellipsoid(z, P);
            plot(x, y, "--r");
            plot(z(1), z(2), "*r");
        end
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


    function c = centroid(self)
        n = self.get_size();
        pts = zeros(2, n);
        for i = 1:n
            pts(:, i) = self.get_state_i(i);
        end
        c = mean(pts, 2);
    end


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


    function remove_state_i(self, idx)
        self.x(2*idx-1:2*idx) = [];
        self.P(2*idx-1:2*idx, :) = [];
        self.P(:, 2*idx-1:2*idx) = [];
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
        self.seeds = seeds;
        self.features = features;
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
            self.z_tmp = z;
            self.R_tmp = R;
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
            new_obs(features(1, :) > 8) = 0;
            % new_obs(self.find_feasile_states(features)) = 0;
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

        self.feat_hist{self.k} = self.features;
        [z, R] = project_features( ...
                        manipulator, ...
                        camera, ...
                        features ...
                        );
        self.z_hist{self.k} = z;
        self.R_hist{self.k} = R;
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


    % function linear_consensus(self, F_other, a_other, q_self, q_other)
    %     self.F = self.F*q_self + F_other*q_other;
    %     self.a = self.a*q_self + a_other*q_other;
    % end

    function KF_update_step(self, N)
        % N = Number of robots
        if size(self.a, 2) > 1
            warning("Invalid size");
        end
        P = inv(inv(self.P) + N*self.F);
        self.x = P * (inv(self.P)*self.x + N*self.a);
        self.P = P;

        self.x_hist{self.k} = self.x;
        self.P_hist{self.k} = self.P;
        self.k = self.k + 1;
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


    function out = redundant_states(self)
        out = [];

        for i = 1:self.get_size()
            xi = self.get_state_i(i);
            Pi = self.get_covariance_i(i);

            if norm(Pi) > 0.1
                continue;
            end

            for j = i+1:self.get_size()
                xj = self.get_state_i(j);

                if mahalanobis_distance(xj, xi, Pi) < 3
                    out = [i, j];
                    return;
                end
            end
        end
    end


    function process_overlapping_states(self, indexes)
        i = indexes(1);
        j = indexes(2);

        x1 = self.get_state_i(i);
        x2 = self.get_state_i(j);
        P1 = self.get_covariance_i(i);
        P2 = self.get_covariance_i(j);

        z = [x1; x2];
        C = blkdiag(P1, P2);
        H = [eye(2); eye(2)];

        Cinv = inv(C);
        P = inv(H' * Cinv * H);
        x = P * H' * Cinv * z;

        self.x(2*i-1:2*i) = x(1:2);
        self.P(2*i-1:2*i, 2*i-1:2*i) = P(1:2, 1:2);

        self.remove_state_i(j);
    end


    function feas = find_feasile_states(self, observations)
        X = [self.x(1:2:end)'; self.x(2:2:end)'];
        feas = zeros(1, size(observations, 2));
        for j = 1:size(observations, 2)
            delta = X - observations(:, j);
            dist = vecnorm(delta);
            if any(dist < 0.2)
                feas(j) = 1;
            end
        end
        feas = logical(feas);
    end % find_feasile_states function


    function [best_idx, best_params, allcosts] = find_best_fit(self, dataset)
        opts = optimset( ...
            "Display", "off", ...
            "MaxIter", 1e4, ...
            "MaxFunEvals", 1e5, ...
            "TolFun", 1e-12, ...
            "TolX", 1e-12 ...
            );
        X = [self.x(1:2:end)'; self.x(2:2:end)'];
        x0 = [mean(X(1,:)); mean(X(2,:)); 0];

        X1 = X - x0(1:2);
        [U1, S1, V1] = svd(X1');
        best_idx = 0;
        best_cost = inf;
        best_params = x0;


        allcosts = zeros(1, length(dataset));
        for i = 1:length(dataset)
            obj = dataset{i};
            poly = obj.get_projected_polygon();
            poly = poly(1:2, 1:end-1);
            poly_mean = mean(poly, 2);
            poly = poly - poly_mean;
            [U2, S2, V2] = svd(poly');
            R = V1 * V2';
            x0(3) = atan2(R(2,1), R(1,1));

            [x, feval] = fminsearch(@(x) self.cost_function(x, obj), x0, opts);
            allcosts(i) = feval;
            if feval < best_cost
                best_cost = feval;
                best_idx = i;
                best_params = x;
            end
        end
    end


    function cost = cost_function(self, x, obj)
        cost_vect = self.cost_function_vector(x, obj);
        good_matches = find(cost_vect < 9);
        delta = size(obj.point_matrix, 2) - length(good_matches);
        cost = mean(cost_vect) * abs(delta);
    end


    function cost = cost_function_vector(self, x, obj)
        test_obj = obj;
        test_obj.RF = rototranslation_matrix(x(1), x(2), x(3));
        points = test_obj.get_projected_polygon();
        points = points(1:2, 1:end-1);
        cost = zeros(1, self.get_size());

        for j = 1:self.get_size()
            delta = points - self.get_state_i(j);
            dist = vecnorm(delta);
            [~, min_idx] = min(dist);
            pt = points(:, min_idx);
            cost(j) = mahalanobis_distance(self.get_state_i(j), pt, self.get_covariance_i(j));
        end
        cost = cost .^ 2;
    end


    function state_to_remove = find_removable_state(self, x, obj)
        costs = self.cost_function_vector(x, obj);
        state_to_remove = [];

        n_vertices = size(obj.point_matrix, 2) - 1;

        [costs, idx] = sort(costs);
        std1 = std(costs);
        std2 = std(costs(1:end-1));

        if self.get_size() > ceil(n_vertices*0.6)
            if std1/std2 > 5
                state_to_remove = idx(end);
            end
        end
    end
end % methods

end % MapEstimator class
