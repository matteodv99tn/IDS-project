classdef Star < BaseObject

properties %% ---- Attributes of the class --------------------------------------------------------

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = Star(orig_RF)
        if nargin == 1
            self.RF = orig_RF;
        else
            self.RF = eye(3);
        end

        % Define the coordinates of the star
        theta = linspace(0, 2*pi, 11); % Angle values for the vertices
        r = [1 0.4 1 0.4 1 0.4 1 0.4 1 0.4 1]; % Distance from the origin for each vertex

        % Calculate the Cartesian coordinates
        x = r .* cos(theta);
        y = r .* sin(theta);
        One = ones(length(r),1);
        starMatrix = [x;y;One'];

        extraColumn = starMatrix(:, 1); % Copy values from first column to last column
        self.point_matrix = [starMatrix extraColumn]; % Append the extracted column at the end

    
        % self.point_matrix = [ ...
        %     -0.5,  0,     0.5,   1,   0.5,  0,  -0.5,  -1,  -0.5; ...
        %     -0.5, -1,    -0.5,   0,   0.5,  1,   0.5,   0,  -0.5; ...
        %      1,    1,     1,     1,   1,    1,   1,     1,   1;...
        %      ];

        self.point_matrix(1:2, :) = 2* self.point_matrix(1:2, :);
        self.name = "Star";

    end % Star constructor


end % methods

end % Star class

