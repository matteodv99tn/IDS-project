classdef Seed < handle

properties %% ---- Attributes of the class --------------------------------------------------------
    
    % We store the data used for the seed generation
    data;
    angles;
    start_index;
    end_index;
   
    % Parameters of a line parametrized as
    %       a*x + b*y + c = 0
    a;
    b;
    c;

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = Seed(in1, in2, in3) 
        % We can have different constructors based on the number of provided inputs, as shown

        if nargin == 3  % seed generation: given a scan and indexes, feet a seed
            self.extract_data_from_scan(in1, in2, in3);
            self.fit_seed();
        elseif nargin == 2 % 2 seeds that should be joined
            self.join_seeds_content(in1, in2);
            self.fit_seed();
        else 
            error("Don't know what to do!")
        end
    end % Seed constructor


    function plot(self)
        P_start = self.predict_point(1);
        P_end   = self.predict_point(size(self.data, 2));
        points  = [P_start, P_end];
        plot(points(1, :), points(2, :), "o-b");
    end % plot function


    function extract_data_from_scan(self, scan, start_index, end_index)
        % Given a scan object and the starting/ending indexes, it extracts the associated data that
        % will be usefull for the seeds (and so for extracting the features).
        self.start_index    = start_index;
        self.end_index      = end_index;
        self.data           = scan.cartesian_points(1:2, start_index:end_index);
        self.angles         = scan.polar_measures(2, start_index:end_index);
    end % extract_data_from_scan function


    function join_seeds_content(self, seed1, seed2)
        % Given two seeds, it joins their information.
        n_overlapping_points = seed1.end_index - seed2.start_index + 1;
        if n_overlapping_points < 0
            warning("Joining seeds with non-overlapping elements");
            n_overlapping_points = 0
        end

        self.start_index    = seed1.start_index;
        self.end_index      = seed2.end_index;
        self.data           = [seed1.data, seed2.data(:, n_overlapping_points+1:end)];
        self.angles         = [seed1.angles, seed2.angles(:, n_overlapping_points+1:end)];
    end % join_seeds_content


    function fit_seed(self, scan, i, j)
        % Given the seeds contents, it computes the line coefficients.
        X = transpose(self.data(1, :));
        Y = transpose(self.data(2, :));
        I = ones(size(self.data, 2), 1);

        if std(X) > std(Y)
            A       = [X, I];
            b       = Y;
            pars    = pinv(A) * b;
            self.a  = pars(1);
            self.b  = -1;
            self.c  = pars(2);
        else
            A       = [Y, I];
            b       = X;
            pars    = pinv(A) * b;
            self.a  = -1;
            self.b  = pars(1);
            self.c  = pars(2);
        end
    end % fit_seed function


    function is_valid = is_valid_seed(self)
        % Determines if the current seed is valid based on the thresholds set in the current 
        % configuration file.
        is_valid = true;

        config  = get_current_configuration();
        delta   = config.feature_extraction.delta;
        epsilon = config.feature_extraction.epsilon;

        for i = 1:size(self.data, 2)
            P_meas  = self.data(:, i);
            P_pred  = self.predict_point(i);

            cond1   = (point_point_distance(P_meas, P_pred) > delta);
            cond2   = (self.distance_from_point(P_meas) > epsilon);
            if cond1 || cond2 
                is_valid = false;
                return;
            end
        end
    end % is_valid_seed function


    function joinable = is_joinable_with(self, other)
        % Determine if a given seed is joinable with another one or not
        config      = get_current_configuration;
        alpha       = config.feature_extraction.max_alpha;
        joinable    = false;

        if other.start_index > self.end_index
            return;
        end

        v1 = [self.a; self.b];
        v2 = [other.a; other.b];
        current_cos = transpose(v1)*v2 / (norm(v1)*norm(v2));

        if abs(current_cos) < cos(alpha) 
            return;
        end

        joinable = true;
    end


    function predicted_point = predict_point(self, idx)
        % Given the seed, it computes the expected measurement associated to the idx-th angle.
        theta   = self.angles(idx);
        den     = self.a*cos(theta) + self.b*sin(theta);

        predicted_point = [ ...
                -self.c * cos(theta) / den; ...
                -self.c * sin(theta) / den  ...
                ];
    end % predict_point function


    function dist = distance_from_point(self, point)
        % Given a point P (that must be specified as the pair of (x,y) coordinates), it computes 
        % the distance from the current seed.
        num     = abs(self.a*point(1) + self.b*point(2) + self.c);
        den     = sqrt(self.a^2 + self.b^2);
        dist    = num / den;
    end % distance_from_point function

    
end % methods

end % Seed class
