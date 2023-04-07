classdef Manipulator < handle


properties %% ---- Attributes of the class --------------------------------------------------------
    
    origin;     % origin of the reference frame as 2D vector (x, y components)
    q;          % joint positions vector
    dq;         % joint velocity vector
    
    % --- Model parameters
    L1;         
    L2;

    % --- Other parameters 
    dt;

    
end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = Manipulator(L1, L2, O)
        % Manipulator object constructor.
        %
        % Parameters:
        %   - L1: length of the first link;
        %   - L2: length of the second link;
        %   - O: origin of the manipulator's reference frame. If not provided, set to (0, 0).

        self.q      = zeros(3, 1);
        self.dq     = zeros(3, 1);
        self.L1     = L1;
        self.L2     = L2;

        if nargin <= 2
            self.origin = zeros(2,1);
        else 
            self.origin = O;
        end
        
        global dt;
        self.dt     = dt;
    end % Manipulator function


    function plot(self)
        OO      = [0; 0; 1];
        delta   = [1; 0; 0];
        RF0     = translation_matrix(self.origin);
        RF1     = RF0 * rotation_matrix(self.q(1));
        RF2     = RF1 * translation_matrix([self.L1; 0]) * rotation_matrix(self.q(2));
        RF3     = RF2 * translation_matrix([self.L2; 0]) * rotation_matrix(self.q(3));
        P1      = RF1 * OO;
        P2      = RF2 * OO;
        P3      = RF3 * OO;
        darrow  = RF3 * delta;
        points  = [P1, P2, P3];

        plot(points(1,:), points(2,:), "-o");
        hold on;
        quiver(P3(1), P3(2), darrow(1), darrow(2), "g");
    end % plot function 


    function update_model(ddq);
    end


end % methods

end % Manipulator class
