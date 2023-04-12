classdef Rectangle < BaseObject

properties %% ---- Attributes of the class --------------------------------------------------------

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = Rectangle(orig_RF)
        if nargin == 1
            self.RF = orig_RF;
        else
            self.RF = eye(3);
        end

        self.point_matrix = [ ...
            -2,  2,  2, -2, -2; ...
            -1, -1,  1,  1, -1; ...
             1,  1,  1,  1,  1  ...
             ];

    end % Rectangle constructor


end % methods

end % Rectangle class

