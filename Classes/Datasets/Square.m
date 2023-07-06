classdef Square < BaseObject

properties %% ---- Attributes of the class --------------------------------------------------------

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = Square(orig_RF)
        if nargin == 1
            self.RF = orig_RF;
        else
            self.RF = eye(3);
        end

        self.point_matrix = [ ...
            -1,  1,  1, -1, -1; ...
            -1, -1,  1,  1, -1; ...
             1,  1,  1,  1,  1  ...
             ];
        self.name = "square";

    end % Square constructor


end % methods

end % Square class

