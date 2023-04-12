classdef Triangle < BaseObject

properties %% ---- Attributes of the class --------------------------------------------------------

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = Triangle(orig_RF)
        if nargin == 1
            self.RF = orig_RF;
        else
            self.RF = eye(3);
        end

        self.point_matrix = [ ...
            -1,  1,  1, 0, -1, -1; ...
            -1, -1,  1, 2,  1, -1; ...
             1,  1,  1, 1,  1,  1;  ...
             ];

    end % Triangle constructor


end % methods

end % Triangle class

