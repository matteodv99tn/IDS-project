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
    end % Manipulator constructor


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
    end % plot function overload


    function update_model(self, ddq)
        % Updates the model based on the provided joint accelerations ddq
        I       = eye(3);
        self.q  = self.q  + I*self.dq*self.dt;
        self.dq = self.dq + I*ddq*self.dt;
    end


    function RF_EE = EE_frame(self)
        % Computes the reference frame of the end effector given the current configuration
        q1  = self.q(1);
        q2  = self.q(2);
        q3  = self.q(3);
        L1  = self.L1;
        L2  = self.L2;
        xo  = self.origin(1);
        yo  = self.origin(2);

        % Computation provided by Maple        
        t1 = cos(q1);
        t2 = cos(q2);
        t4 = sin(q1);
        t5 = sin(q2);
        t7 = t2 * t1 - t5 * t4;
        t8 = cos(q3);
        t12 = -t5 * t1 - t2 * t4;
        t13 = sin(q3);
        res__1_1 = t13 * t12 + t8 * t7;
        t18 = -t8 * t12;
        res__1_2 = -t13 * t7 - t18;
        t20 = t2 * L2 + L1;
        res__1_3 = -t4 * t5 * L2 + t1 * t20 + xo;
        res__2_1 = t13 * t7 + t18;
        res__2_2 = res__1_1;
        res__2_3 = t5 * t1 * L2 + t4 * t20 + yo;
        res__3_3 = 1;
        RF_EE = zeros(3, 3);
        RF_EE(1, 1) = res__1_1;
        RF_EE(1, 2) = res__1_2;
        RF_EE(1, 3) = res__1_3;
        RF_EE(2, 1) = res__2_1;
        RF_EE(2, 2) = res__2_2;
        RF_EE(2, 3) = res__2_3;
        RF_EE(3, 3) = res__3_3;
    end


    function J_EE = EE_jacobian(self)
        % Jacobian of the end effector (wrt the joints) given the current robot configuration
        q1  = self.q(1);
        q2  = self.q(2);
        q3  = self.q(3);
        L1  = self.L1;
        L2  = self.L2;

        % Computations from Maple
        t1 = cos(q2);
        t3 = -t1 * L2 - L1;
        t4 = sin(q1);
        t6 = cos(q1);
        t8 = sin(q2);
        res__1_1 = -t8 * t6 * L2 + t4 * t3;
        res__1_2 = -L2 * (t1 * t4 + t8 * t6);
        res__2_1 = -t4 * t8 * L2 - t6 * t3;
        res__2_2 = L2 * (t1 * t6 - t8 * t4);
        res__3_1 = 1;
        res__3_2 = 1;
        res__3_3 = 1;
        J_EE = zeros(3, 3);
        J_EE(1, 1) = res__1_1;
        J_EE(1, 2) = res__1_2;
        J_EE(2, 1) = res__2_1;
        J_EE(2, 2) = res__2_2;
        J_EE(3, 1) = res__3_1;
        J_EE(3, 2) = res__3_2;
        J_EE(3, 3) = res__3_3;
    end


end % methods

end % Manipulator class
