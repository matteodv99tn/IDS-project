classdef BaseObject

properties %% ---- Attributes of the class --------------------------------------------------------

    RF;                         % reference frame of the object
    point_matrix;               % 3xN matrix storing the vertices of the closed polygon (first and
                                % last point should be the same). Row 1 -> x coords, row 2 -> y
                                % coords, row 3 -> ones.
    name;

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = BaseObject(RF)
        % set RF optional
        self.name = "";
    end % Object constructor


    function plot(self)
        data = self.RF * self.point_matrix;
        plot(data(1,:), data(2,:), "k");
    end % plot function


    function poly = get_projected_polygon(self)
        poly = self.RF * self.point_matrix;
    end % get_projected_polygon function


end % methods

end % Object class
