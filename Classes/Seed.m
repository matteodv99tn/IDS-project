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
        P_start = self.predict_point(self.angles(1));
        P_end   = self.predict_point(self.angles(end));
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


    function grow(self, scan)

        i = self.start_index - 1;
        can_grow = true;
        while can_grow && i >= 1
            curr_point = scan.cartesian_points(1:2, i);
            curr_angle = scan.polar_measures(2, i);
            
            if self.point_is_compatible(curr_point, curr_angle)
                i = i - 1;
            else
                can_grow = false;
            end
        end
        i = i+1;

        if i ~= self.start_index 
            new_data = i:self.start_index - 1;
            self.data = [scan.cartesian_points(1:2, new_data), self.data];
            self.angles = [scan.polar_measures(2, new_data), self.angles];
            self.start_index = i;
        end
        
        i = self.end_index + 1;
        can_grow = true;
        while can_grow && i <= size(scan.cartesian_points, 2)
            curr_point = scan.cartesian_points(1:2, i);
            curr_angle = scan.polar_measures(2, i);
            
            if self.point_is_compatible(curr_point, curr_angle)
                i = i + 1;
            else
                can_grow = false;
            end
        end
        i = i-1;

        if i ~= self.start_index 
            new_data = self.end_index+1:i;
            self.data = [self.data, scan.cartesian_points(1:2, new_data)];
            self.angles = [self.angles, scan.polar_measures(2, new_data)];
            self.end_index = i;
        end

        self.fit_seed();
    end


    function P = compute_intersection(self, other) 
        % Computes the point of intersection between two seeds, implicitly assuming that they are 
        % overlapping (this check is not performed).
        A = [ self.a,    self.b; ...
              other.a,   other.b; ...
              ];
        b = [ -self.c; ...
              -other.c ...
              ];
        P = linsolve(A, b);
    end


    function P = get_startpoint(self)
        % Returns the starting point of the segment
        P = self.predict_point(self.angles(1));
    end


    function P = get_endpoint(self)
        % Returns the ending point of the segment
        P = self.predict_point(self.angles(end));
    end


    function len = length(self)
        % Computes the length of the seeds based on its endpoints
        len = point_point_distance(self.get_startpoint(), self.get_endpoint());
    end


    function fit_seed(self)
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

        for i = 1:size(self.data, 2)

            if ~self.point_is_compatible(self.data(:, i), self.angles(i));
                is_valid = false;
                return;
            end
        end
    end % is_valid_seed function


    function is_compatible = point_is_compatible(self, point, angle)
        config      = get_current_configuration();
        delta       = config.feature_extraction.delta;
        epsilon     = config.feature_extraction.epsilon;

        point_pred  = self.predict_point(angle);

        cond1       = (point_point_distance(point, point_pred) <= delta);
        cond2       = (self.distance_from_point(point) <= epsilon);
        is_compatible = cond1 || cond2;
    end


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


    function predicted_point = predict_point(self, theta)
        % Given the seed, it computes the expected measurement associated to the idx-th angle.
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
