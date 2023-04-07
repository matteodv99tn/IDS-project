classdef Object < handle

properties %% ---- Attributes of the class --------------------------------------------------------
    
    RF;                         % reference frame of the object
    point_matrix;               % 3xN matrix storing the vertices of the closed polygon (first and
                                % last point should be the same). Row 1 -> x coords, row 2 -> y 
                                % coords, row 3 -> ones.

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = Object(RF) 
        % set RF optional
    end % Object constructor


    function plot(self)
    end % plot function 


end % methods

end % Object class
