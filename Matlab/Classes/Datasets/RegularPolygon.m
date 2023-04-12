classdef RegularPolygon < BaseObject

properties %% ---- Attributes of the class --------------------------------------------------------

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = RegularPolygon(nvertices, RF)

        angles = linspace(0, 2*pi, nvertices+1);
        if nargin == 1
            self.RF = eye(3);
        else
            self.RF = RF;
        end

        self.point_matrix = [ ...
            cos(angles); ...
            sin(angles); ...
            eye(1, nvertices+1)  ...
            ];

    end % RegularPolygon constructor


end % methods

end % RegularPolygon class

