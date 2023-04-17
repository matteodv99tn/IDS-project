classdef Manipulator < handle


properties %% ---- Attributes of the class --------------------------------------------------------
    
    origin;     % origin of the reference frame as 2D vector (x, y components)
    q;          % joint positions vector
    dq;         % joint velocity vector
    d_true;     % true joint states; stored differently since q, dq will already embedd the 
    dq_true;    % measurement noise 

    R_q;        % covariance matrix of the joint position noise
    R_dq;       % covariance matrix of the joint velocity noise
    R;          % overall covariance matrix
    
    % --- Model parameters
    L1;         
    L2;

    % --- Controller 
    controller;

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
        config = get_current_configuration();

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

        self.R_q    = eye(3) * config.manipulator.std_position;
        self.R_dq   = eye(3) * config.manipulator.std_velocity;
        self.R      = blkdiag(self.R_q, self.R_dq);

        self.controller = BaseController();
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


    function set_controller(self, controller)
        self.controller = controller;
    end


    function update_control_law(self);
        tau = self.controller.compute_tau(self);
        self.update_model(tau);
    end


    function update_model(self, tau)
        % Updates the model based on the provided joint accelerations ddq
        M = self.mass_matrix();
        h = self.bias_forces();

        ddq          = linsolve(M, tau - h);
        self.dq_true = self.dq + ddq*self.dt;
        self.q_true  = self.q  + self.dq*self.dt;
        self.q       = self.q_true + mvnrnd(zeros(3,1), self.R_q)';
        self.dq      = self.dq_true + mvnrnd(zeros(3,1), self.R_dq)';
    end


    function EE = get_EE_state(self)
        % Return the sate of the end effector, i.e. the cartesian coordinates wrt world as well as
        % it's heading.
        EE = self.EE_frame() * [0; 0; 1];
        EE(3) = sum(self.q);
    end


    function v = get_EE_velocity(self)
        % Returns the velocity of the end effector in cartesian coordinate
        v = self.EE_jacobian() * self.dq;
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

        % Maple generated code 
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

        % Maple generated code 
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


    function M = mass_matrix(self)
        % Computes the mass matrix of the current configuration
        q1 = self.q(1);
        q2 = self.q(2);
        q3 = self.q(3);
        L1 = self.L1;
        L2 = self.L2;

        % Maple generated code
        t1 = L2 ^ 2;
        t2 = cos(q3);
        t7 = cos(q2);
        t8 = t7 * L1 * (t1 + 2 * t2 + 2 * L2);
        t9 = t2 * L2;
        t10 = 2 * t9;
        t11 = sin(q2);
        t12 = sin(q3);
        t14 = L1 * t12 * t11;
        t16 = L1 ^ 2;
        t24 = t1 * L2 / 3;
        res__1_1 = t8 + t10 - 2 * t14 + t16 * L1 / 3 + t16 * (3 * L2 + 3) / 3 + t24 + t1 + 1;
        res__1_2 = t8 / 2 + t24 - t14 + t10 + t1 + 1;
        res__1_3 = -t14 + 1 + t2 * (L1 * t7 + L2);
        res__2_1 = res__1_2;
        res__2_2 = t24 + t1 + t10 + 1;
        res__2_3 = t9 + 1;
        res__3_1 = res__1_3;
        res__3_2 = res__2_3;
        res__3_3 = 1;
        M = zeros(3, 3);
        M(1, 1) = res__1_1;
        M(1, 2) = res__1_2;
        M(1, 3) = res__1_3;
        M(2, 1) = res__2_1;
        M(2, 2) = res__2_2;
        M(2, 3) = res__2_3;
        M(3, 1) = res__3_1;
        M(3, 2) = res__3_2;
        M(3, 3) = res__3_3;
    end


    function h = bias_forces(self)
        % Computes the mass matrix of the current configuration
        q1 = self.q(1);
        q2 = self.q(2);
        q3 = self.q(3);
        q1__vel = self.dq(1);
        q2__vel = self.dq(2);
        q3__vel = self.dq(3);
        L1 = self.L1;
        L2 = self.L2;

        % Maple generated code
        t1 = q2__vel / 2;
        t2 = q3__vel / 2;
        t3 = q1__vel + t1 + t2;
        t4 = q2__vel + q3__vel;
        t6 = cos(q3);
        t16 = sin(q2);
        t18 = sin(q3);
        t20 = cos(q2);
        t25 = (q1__vel + q2__vel + t2) * L2 * q3__vel;
        res__1_1 = -t16 * (2 * t6 * t4 * t3 + L2 * (q1__vel + t1) * q2__vel * (L2 + 2)) * L1 - 2 * (t20 * t4 * t3 * L1 + t25) * t18;
        t29 = q1__vel ^ 2;
        t30 = t29 * L1;
        t31 = t20 * t30;
        t37 = L2 ^ 2;
        res__2_1 = t18 * (2 * t31 - 4 * t25) / 2 + (t37 + 2 * t6 + 2 * L2) * t29 * t16 * L1 / 2;
        t45 = (q1__vel + q2__vel) ^ 2;
        res__3_1 = t18 * (t45 * L2 + t31) + t6 * t16 * t30;
        h = zeros(3, 1);
        h(1, 1) = res__1_1;
        h(2, 1) = res__2_1;
        h(3, 1) = res__3_1;
    end


end % methods

end % Manipulator class
