classdef Dart < BaseObject

properties %% ---- Attributes of the class --------------------------------------------------------

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = Dart(orig_RF)
        if nargin == 1
            self.RF = orig_RF;
        else
            self.RF = eye(3);
        end

        self.point_matrix = [ ...
            -0.5,  0,     0.5,   0,   -0.5; ...
            -0.5, -0.25,  -0.5,  0.5, -0.5; ...
             1,    1,     1,     1,      1;  ...
             ];
        self.point_matrix(1:2, :) = 2* self.point_matrix(1:2, :);
        self.name = "dart";


    end % Dart constructor


end % methods

end % Dart class

