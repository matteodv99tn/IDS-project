classdef Manipulator < handle


properties %% ---- Attributes of the class --------------------------------------------------------
    
    origin;     % origin of the reference frame as 2D vector (x, y components)
    q_est;      % joint positions vector estimate
    dq_est;     % joint velocity vector estimate
    q_true;     % true joint states; stored differently since q, dq will already embedd the 
    dq_true;    % me asurement noise 
    P;

    Q_tau;      % covariance matrix on joint torque measure
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

        self.q_est      = zeros(3, 1);
        self.dq_est     = zeros(3, 1);
        self.q_true     = zeros(3, 1);
        self.dq_true    = zeros(3, 1);
        self.P          = zeros(6, 6);
        self.L1         = L1;
        self.L2         = L2;

        if nargin <= 2
            self.origin = zeros(2,1);
        else 
            self.origin = O;
        end
        
        global dt;
        self.dt     = dt;
        
        self.Q_tau  = eye(3) * 10;
        self.R_q    = eye(3) * config.manipulator.std_position;
        self.R_dq   = eye(3) * config.manipulator.std_velocity;
        self.R      = blkdiag(self.R_q, self.R_dq);

        self.controller = BaseController();
    end % Manipulator constructor


    function plot(self)
        OO      = [0; 0; 1];
        delta   = [1; 0; 0];
        RF0     = translation_matrix(self.origin);
        RF1     = RF0 * rotation_matrix(self.q_true(1));
        RF2     = RF1 * translation_matrix([self.L1; 0]) * rotation_matrix(self.q_true(2));
        RF3     = RF2 * translation_matrix([self.L2; 0]) * rotation_matrix(self.q_true(3));
        P1      = RF1 * OO;
        P2      = RF2 * OO;
        P3      = RF3 * OO;
        darrow  = RF3 * delta;
        points  = [P1, P2, P3];

        plot(points(1,:), points(2,:), "-o");
        hold on;
        quiver(P3(1), P3(2), darrow(1), darrow(2), "g");
    end % plot function overload


    function set_initial_joint_config(self, q0)
        % Sets the initial joint configuration to the provided value
        self.q_est  = q0;
        self.q_true = q0;
        self.P      = zeros(6, 6);
    end

    function set_controller(self, controller)
        self.controller = controller;
    end


    function update_kinematic_dynamics(self) 

        % Compute torque 
        tau_applied = self.controller.compute_tau(self)
        tau_meas    = tau_applied + transpose(mvnrnd(zeros(3,1), self.Q_tau));

        % Apply torque and update dynamics
        M = self.mass_matrix(false);
        h = self.bias_forces(false);
        qdd = linsolve(M, tau_applied - h);
        self.dq_true = self.qd_true + qdd*self.dt;
        self.q_true = self.q_true + self.dq_true*self.dt;

        % KF prediction step
        M = self.mass_matrix(true);
        h = self.bias_forces(true);
        qdd = linsolve(M, tau_measured - h);
        self.dq_est = self.qd_est + qdd*self.dt;
        self.q_est = self.q_est + self.dq_est*self.dt;
        A = self.jacobian_dynamic_states(true);
        B = self.jacobian_dynamic_inputs(true);
        self.P = A*self.P*transpose(A) + B*self.Q_tau*transpose(B);

        % KF update step
        q_meas = self.q_true + transpose(mvnrnd(zeros(3,1), self.R_q));
        x = [self.q_est; self.dq_est];
        H = zeros(3, 6);
        H(:, 1:3) = eye(3);
        S = H*self.P*transpose(H) + self.R_q;
        W = self.P * transpose(H) * inverse(S);
        x = x + W(q_meas - H*x)
        self.P = (eye(6) - W*H) * self.P;
        self.q_est = x(1:3);
        self.dq_est = x(4:6);
    end


    function EE = get_EE_state(self, estimated)
        % Return the sate of the end effector, i.e. the cartesian coordinates wrt world as well as
        % it's heading.
        if estimated
            q = self.q_est;
        else
            q = self.q_true;
        end
        EE = self.EE_frame(estimated) * [0; 0; 1];
        EE(3) = sum(q);
    end


    function v = get_EE_velocity(self, estimated)
        % Returns the velocity of the end effector in cartesian coordinate
        if estimated
            dq = self.dq_est;
        else
            dq = self.dq_true;
        end
        v = self.EE_jacobian(estimated) * dq;
    end

    function RF_EE = EE_frame(self, estimated)
        % Computes the reference frame of the end effector given the current configuration
        if estimated  
            q = self.q_est;
        else 
            q = self.q_true;
        end
        q1  = q(1);
        q2  = q(2);
        q3  = q(3);
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


    function J_EE = EE_jacobian(self, estimated)
        % Jacobian of the end effector (wrt the joints) given the current robot configuration
        if estimated  
            q = self.q_est;
        else 
            q = self.q_true;
        end
        q1  = q(1);
        q2  = q(2);
        q3  = q(3);
        L1  = self.L1;
        L2  = self.L2;

        % Maple generated code 
        t1 = cos(q2);
        t3 = -t1 * L2 - L1;
        t4 = sin(q1);
        t6 = sin(q2);
        t7 = t6 * L2;
        t8 = cos(q1);
        res__1_1 = t4 * t3 - t8 * t7;
        res__1_2 = -(t1 * t4 + t8 * t6) * L2;
        res__2_1 = -t8 * t3 - t4 * t7;
        res__2_2 = (t8 * t1 - t4 * t6) * L2;
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


    function M = mass_matrix(self, estimated)
        % Computes the mass matrix of the current configuration
        if estimated  
            q = self.q_est;
        else 
            q = self.q_true;
        end
        q1 = q(1);
        q2 = q(2);
        q3 = q(3);
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


    function h = bias_forces(self, estimated)
        % Computes the mass matrix of the current configuration
        if estimated  
            q = self.q_est;
            dq = self.dq_est;
        else 
            q = self.q_true;
            dq = self.dq_true;
        end
        q1 = q(1);
        q2 = q(2);
        q3 = q(3);
        q1__vel = dq(1);
        q2__vel = dq(2);
        q3__vel = dq(3);
        L1 = self.L1;
        L2 = self.L2;

        % Maple generated code
        t2 = q2__vel / 2;
        t3 = q3__vel / 2;
        t5 = (q1__vel + t2 + t3) * (q2__vel + q3__vel);
        t6 = cos(q3);
        t16 = sin(q2);
        t18 = sin(q3);
        t19 = cos(q2);
        t24 = (q1__vel + q2__vel + t3) * L2 * q3__vel;
        res__1_1 = -t16 * L1 * (2 * t6 * t5 + L2 * q2__vel * (L2 + 2) * (q1__vel + t2)) - 2 * (t19 * L1 * t5 + t24) * t18;
        t28 = q1__vel ^ 2;
        t29 = t28 * L1;
        t30 = t19 * t29;
        t36 = L2 ^ 2;
        res__2_1 = t18 * (2 * t30 - 4 * t24) / 2 + (t36 + 2 * t6 + 2 * L2) * t28 * t16 * L1 / 2;
        t44 = (q1__vel + q2__vel) ^ 2;
        res__3_1 = t18 * (t44 * L2 + t30) + t6 * t16 * t29;
        h = zeros(3, 1);
        h(1, 1) = res__1_1;
        h(2, 1) = res__2_1;
        h(3, 1) = res__3_1;
    end


    function A = jacobian_dynamic_states(self, estimated)
        if estimated  
            q = self.q_est;
            dq = self.dq_est;
        else 
            q = self.q_true;
            dq = self.dq_true;
        end
        q1 = q(1);
        q2 = q(2);
        q3 = q(3);
        q1__vel = dq(1);
        q2__vel = dq(2);
        q3__vel = dq(3);
        L1 = self.L1;
        L2 = self.L2;

        % Maple generated code
        t1 = q1__vel + q2__vel;
        t2 = t1 ^ 2;
        t3 = t2 * L2;
        t4 = cos(q3);
        t5 = t4 ^ 2;
        t6 = t5 ^ 2;
        t9 = q1__vel + q2__vel + q3__vel;
        t10 = t9 ^ 2;
        t11 = t5 * t4;
        t12 = t11 * t10;
        t19 = L2 + 2;
        t20 = t19 * t10;
        t23 = 0.8e1 / 0.3e1 + L2;
        t24 = t23 * L2;
        t25 = L2 + 3;
        t26 = t25 * t2;
        t30 = cos(q2);
        t31 = t30 ^ 2;
        t32 = t31 * t30;
        t35 = q1__vel ^ 2;
        t36 = t35 * L1;
        t38 = L1 + 0.3e1 / 0.2e1 * L2;
        t40 = t6 * t38 * t36;
        t42 = L2 ^ 2;
        t43 = sin(q2);
        t44 = t43 * t42;
        t45 = sin(q3);
        t46 = t2 * t45;
        t50 = L2 * t45;
        t52 = t43 * t10 * t50;
        t53 = 0.3e1 / 0.4e1 * t52;
        t55 = L1 + 0.60e2 / 0.13e2;
        t57 = 0.36e2 / 0.13e2 * L1;
        t66 = t43 * t2;
        t70 = 0.15e2 / 0.8e1 * t42;
        t71 = L1 + 6;
        t72 = L2 * t71;
        t73 = 3 * L1;
        t74 = t70 + t72 + t73;
        t76 = L1 * t35 * t74;
        t82 = L1 * L2;
        t86 = t35 * t45;
        t87 = t43 * t38;
        t95 = L1 - 0.3e1 / 0.4e1;
        t101 = t43 * t36;
        t103 = 0.3e1 / 0.2e1 * t42;
        t104 = L1 - 3;
        t105 = L2 * t104;
        t106 = -t103 + t105 + t73;
        t107 = t10 * t106;
        t117 = t43 * L2;
        t118 = t45 * t117;
        t119 = L1 + L2;
        t124 = t10 * t45;
        t128 = t43 * (L1 + 4 * L2) * t124;
        t142 = t43 * t45;
        t143 = t10 * t142;
        t148 = 3 * L2;
        t149 = t148 + L1;
        t158 = t23 ^ 2;
        t161 = t31 ^ 2;
        t168 = t43 * t4;
        t172 = 4 * L1;
        t189 = 6 * L2;
        t191 = t5 * (-t73 - t189);
        t192 = t149 * t25;
        t193 = t191 + t192;
        t196 = t30 * t168;
        t199 = t193 ^ 2;
        t202 = 0.1e1 / L1;
        t203 = t202 / (0.81e2 / 0.16e2 * t161 * (0.32e2 / 0.9e1 * t6 + t5 * (-0.8e1 / 0.3e1 * L2 - 0.80e2 / 0.9e1) + t158) * t42 + 0.27e2 / 0.2e1 * t32 * t168 * t45 * t42 * (-0.4e1 / 0.3e1 * t5 + L2 + 0.8e1 / 0.3e1) - 0.9e1 / 0.2e1 * t31 * (t6 * (t172 + 10 * L2) + t5 * (-10 * t42 + (-0.13e2 / 0.3e1 * L1 - 30) * L2 - 12 * L1) + t149 * t25 * t23) * L2 - 6 * t196 * t45 * t193 * L2 + t199);
        res__1_2 = 6 * t203 * (0.9e1 / 0.16e2 * t32 * (0.8e1 / 0.3e1 * t6 * t3 - 0.4e1 / 0.3e1 * t12 - 0.7e1 / 0.3e1 * t5 * t2 * L2 * (L2 + 0.24e2 / 0.7e1) + 2 * t4 * t20 + t26 * t24) * L2 + t31 * (3 * t40 - 0.3e1 / 0.2e1 * t11 * t46 * t44 + t5 * (t53 - 0.13e2 / 0.4e1 * (0.21e2 / 0.13e2 * t42 + L2 * t55 + t57) * t36) + 0.21e2 / 0.16e2 * t4 * t66 * t45 * (L2 + 0.20e2 / 0.7e1) * t42 + 0.3e1 / 0.4e1 * (t53 + t76) * t23) + t30 * (0.3e1 / 0.4e1 * t6 * t2 * t82 - 3 * t11 * (t87 * t86 + t10 / 2) * L1 - t5 * (-0.3e1 / 0.8e1 * t42 + L2 * t95 + t73) * t3 + t4 * (t101 * t45 * t74 + t107 / 2) + t26 * t106 * L2 / 4) - 0.3e1 / 0.2e1 * t40 - 0.3e1 / 0.4e1 * t11 * t119 * t2 * t118 + t5 * (-0.3e1 / 0.4e1 * t128 + 0.13e2 / 0.8e1 * L1 * t35 * (0.30e2 / 0.13e2 * t42 + t72 + t57)) + t4 * t25 * t46 * t43 * t82 / 4 - 0.3e1 / 0.8e1 * t149 * t25 * (-0.2e1 / 0.3e1 * t143 + t23 * t36)) * L2;
        t206 = L2 + 4;
        t209 = t10 * L2;
        t218 = 0.3e1 / 0.5e1 * t52;
        t221 = 0.3e1 / 0.2e1 * L1;
        t235 = t143 * L2 * (L2 + 0.10e2 / 0.3e1);
        t240 = t10 * L1;
        t243 = L1 - 0.3e1 / 0.2e1;
        t252 = 0.12e2 / 0.5e1 * L1;
        t256 = 3 * t42;
        t257 = t256 + t105 - t73;
        t258 = t10 * t257;
        t266 = 2 * L2;
        t267 = L1 + t266;
        t268 = t43 * t267;
        t271 = t256 + t72 + t221;
        res__1_3 = -0.15e2 / 0.2e1 * t203 * (0.3e1 / 0.10e2 * t32 * (2 * t12 + t5 * t206 * t3 - 0.3e1 / 0.2e1 * t4 * t209 - t206 * t3 / 2) * L2 + t31 * (t5 * (-t218 + 0.4e1 / 0.5e1 * L1 * t35 * (t70 + (L1 + 0.15e2 / 0.4e1) * L2 + t221)) - 0.3e1 / 0.10e2 * t4 * t206 * t2 * t45 * t44 - 0.9e1 / 0.10e2 * t235 - 0.2e1 / 0.5e1 * t76) + t30 * (0.3e1 / 0.5e1 * t11 * t240 + 0.2e1 / 0.5e1 * t5 * (L2 * t243 + t221) * t3 + t4 * (t101 * t45 * (0.6e1 / 0.5e1 * t42 + (0.12e2 / 0.5e1 + L1) * L2 + t252) + t258 / 5) - t26 * t82 / 5) + t5 * (0.6e1 / 0.5e1 * t268 * t124 - 0.2e1 / 0.5e1 * L1 * t35 * t271) + 0.4e1 / 0.5e1 * t168 * t38 * t25 * t2 * t50 + t149 * t25 * (2 * t143 + t36) / 5) * L2;
        t291 = L1 * t30;
        t293 = t1 * L2;
        t303 = t30 * t45;
        t304 = t303 * t293;
        t308 = t45 * L1;
        t313 = L1 * q1__vel;
        t321 = t43 * t25 * t293;
        t325 = L2 * t31;
        t332 = 3 * t45 * t168 * t30 * L2;
        t333 = 0.9e1 / 0.4e1 * t42;
        t335 = t31 * (-t333 - t189);
        t338 = 0.1e1 / (t5 * (3 * t325 - t73 - t189) - t332 + t335 + t192) * t202;
        res__1_4 = 0.9e1 / 0.2e1 * t338 * (-0.4e1 / 0.3e1 * t5 * (q1__vel * t291 + t293 / 2) * t43 + t4 * (-0.4e1 / 0.3e1 * q1__vel * t45 * t31 * L1 - 0.2e1 / 0.3e1 * t304 + 0.4e1 / 0.3e1 * t43 * t9 + 0.2e1 / 0.3e1 * q1__vel * t308) + t30 * (t43 * t23 * t313 - 0.2e1 / 0.3e1 * t9 * t45) + 0.2e1 / 0.3e1 * t321) * L2;
        t342 = 2 * t9;
        res__1_5 = 3 * t338 * (-t5 * t1 * t117 + t4 * (t43 * t342 - t304) + t321 - t9 * t303) * L2;
        t350 = t9 * L2;
        t354 = t5 * L2;
        t359 = 0.1e1 / (t31 * (3 * t354 - t333 - t189) - t332 + t191 + t192);
        res__1_6 = 6 * t359 * t202 * (t168 - t303 / 2) * t350;
        t362 = t42 * L2;
        t363 = t362 * t2;
        t365 = L1 ^ 2;
        t366 = L2 * t365;
        t367 = t35 * t366;
        t369 = t365 * L1;
        t370 = t35 * t369;
        t374 = t10 * t42;
        t377 = t42 ^ 2;
        t378 = t377 * t2;
        t381 = t42 * t365;
        t382 = t35 * t381;
        t384 = t35 * t365;
        t385 = t72 * t384;
        t396 = t378 + 3 * t363 + 3 * t382 + t385 + 2 * t370;
        t402 = q1__vel * q2__vel;
        t403 = q2__vel ^ 2;
        t405 = t35 + t402 + t403 / 2;
        t406 = L2 * t405;
        t407 = L1 * t38;
        t408 = t6 * t407;
        t413 = t363 / 2 + 0.3e1 / 0.2e1 * t367 + t370;
        t414 = t413 * t45;
        t429 = t35 + 0.9e1 / 0.11e2 * t402 + 0.9e1 / 0.22e2 * t403;
        t440 = 3 * L1 * (t35 + 0.10e2 / 0.11e2 * t402 + 0.5e1 / 0.11e2 * t403);
        t448 = 2 * t363;
        t451 = (0.24e2 / 0.5e1 + L1) * L2;
        t457 = L1 + 0.3e1 / 0.2e1;
        t458 = L2 * t457;
        t470 = t406 + 0.7e1 / 0.3e1 * t35 + 2 * t402 + t403;
        t482 = L1 * t42;
        t491 = t43 * t209;
        t495 = t377 * L2 * t2;
        t497 = t95 * t2;
        t505 = L1 - 1;
        t513 = t365 ^ 2;
        t514 = t35 * t513;
        t546 = L2 * t308;
        t547 = t43 * t149;
        t579 = t378 / 2;
        t580 = 0.3e1 / 0.2e1 * t363;
        t583 = 3 * t42 * t457 * t36;
        t586 = (0.21e2 / 0.2e1 + L1) * L2 * t384;
        t587 = 3 * t370;
        t592 = t149 * (L2 + 1);
        t606 = 0.1e1 / L2;
        res__2_2 = t203 * t606 * (-54 * t32 * (t6 * (0.8e1 / 0.3e1 * t363 + 8 * t367 + 0.16e2 / 0.3e1 * t370) - 0.4e1 / 0.3e1 * t11 * t374 + t5 * (-0.7e1 / 0.3e1 * t378 - 8 * t363 - 7 * t382 - 0.10e2 / 0.3e1 * t385 - 0.32e2 / 0.3e1 * t370) + 2 * t4 * t19 * t374 + t396 * t23) * L2 - 180 * t31 * (0.16e2 / 0.5e1 * t408 * t406 + t11 * (-0.8e1 / 0.5e1 * t43 * t414 - 0.4e1 / 0.5e1 * t10 * t365) - 0.44e2 / 0.15e2 * t5 * (-0.3e1 / 0.22e2 * t52 + L1 * (t42 * (0.69e2 / 0.44e2 * t35 + 0.27e2 / 0.22e2 * t402 + 0.27e2 / 0.44e2 * t403) + L2 * (L1 * t429 + 0.105e3 / 0.22e2 * t35 + 0.45e2 / 0.11e2 * t402 + 0.45e2 / 0.22e2 * t403) + t440)) * L2 + t4 * (t43 * (0.7e1 / 0.10e2 * t378 + t448 + 0.21e2 / 0.10e2 * t382 + t451 * t384 + 0.12e2 / 0.5e1 * t370) * t45 + 0.4e1 / 0.5e1 * t10 * L1 * (t70 + t458 + L1)) + 0.4e1 / 0.5e1 * (0.3e1 / 0.8e1 * t143 * t24 + t470 * L1 * t74) * L2) * L2 + t30 * (-144 * t6 * t413 * L1 + 576 * t11 * (t87 * t405 * t45 + t10 / 4) * t482 + t5 * (-144 * t491 * t45 * t365 - 36 * t495 + 96 * t377 * t497 - 108 * t362 * (t36 - 0.8e1 / 0.3e1 * t2) * L1 + 216 * t42 * t505 * t384 + 120 * (L1 + 0.21e2 / 0.5e1) * L2 * t370 + 288 * t514) - 312 * t4 * (t43 * L1 * t45 * (t42 * (0.27e2 / 0.26e2 * t403 + 0.21e2 / 0.13e2 * t35 + 0.27e2 / 0.13e2 * t402) + L2 * (L1 * (0.18e2 / 0.13e2 * t402 + t35 + 0.9e1 / 0.13e2 * t403) + 0.72e2 / 0.13e2 * t402 + 0.60e2 / 0.13e2 * t35 + 0.36e2 / 0.13e2 * t403) + 0.36e2 / 0.13e2 * L1 * (t35 + 0.4e1 / 0.3e1 * t402 + 0.2e1 / 0.3e1 * t403)) + 0.2e1 / 0.13e2 * t107) * t42 - 72 * t547 * t20 * t546 - 24 * t106 * t396) + 288 * t408 * t405 * t42 + t11 * (144 * t43 * t119 * t414 - 72 * t10 * t366) - 264 * t5 * (-0.3e1 / 0.11e2 * t128 + (t42 * (0.24e2 / 0.11e2 * t35 + 0.18e2 / 0.11e2 * t402 + 0.9e1 / 0.11e2 * t403) + L2 * t71 * t429 + t440) * L1) * t42 - 48 * t4 * L1 * (t43 * (t579 + t580 + t583 + t586 + t587) * t45 - 0.3e1 / 0.2e1 * t592 * t209) + 72 * t192 * (-t143 / 3 + t470 * L1) * t42) / 16;
        t618 = t149 * L1;
        t624 = L1 * t362;
        t625 = t19 * t2;
        t627 = t161 * t625 * t624;
        t631 = 0.3e1 / 0.2e1 * t382;
        t632 = L2 * t369;
        t633 = t35 * t632;
        t634 = 4 * t370;
        t643 = t42 * (3 * t35 + 9 * t402 + 0.9e1 / 0.2e1 * t403);
        t645 = q1__vel + 0.3e1 / 0.5e1 * q2__vel;
        t647 = q1__vel + 3 * q2__vel;
        t650 = 6 * t35;
        t657 = t35 + 3 * t402 + 0.3e1 / 0.2e1 * t403;
        t669 = t43 * t240;
        t675 = t2 * L1;
        t676 = t362 * t675;
        t694 = L2 * t657 + t675;
        t715 = L1 - 0.6e1 / 0.5e1;
        t736 = 0.24e2 / 0.7e1 * t35;
        t754 = L1 + 0.9e1 / 0.2e1;
        t756 = t362 * t2 * t754;
        t765 = L1 + 3;
        t778 = L2 + 0.4e1 / 0.3e1;
        t819 = 45 * t42;
        t822 = t267 ^ 2;
        t845 = t335 + t192;
        t849 = t845 ^ 2;
        t851 = 0.1e1 / (t6 * (18 * t161 * t42 + t31 * (-18 * t82 - t819) + 9 * t822) + 18 * t11 * (-t325 + L1 + t266) * t30 * t118 + t5 * (t161 * (-0.27e2 / 0.2e1 * t362 - t819) + t31 * (45 * t362 + t42 * (135 + 0.39e2 / 0.2e1 * L1) + 54 * t82) - 6 * t267 * t192) - 6 * t196 * t845 * t50 + t849);
        res__2_3 = t202 * t851 * t606 * (144 * t11 * t10 * (3 * t161 * t482 + t32 * t362 / 2 - 0.5e1 / 0.2e1 * t31 * t149 * t82 + t30 * t482 / 2 + t267 * t618) + t5 * (216 * t627 - 72 * t32 * (6 * t491 * t308 - t448 - t579 - t631 + t633 + t634) * L2 - 120 * t31 * (t218 + L1 * (t643 + L2 * (L1 * t647 * t645 + 18 * t402 + 9 * t403 + t650) + 0.12e2 / 0.5e1 * t657 * L1)) * t42 + t30 * (360 * t669 * (L1 + 0.12e2 / 0.5e1 * L2) * t50 + 48 * t377 * t243 * t2 + 72 * t676 + 288 * t42 * (L1 + 0.3e1 / 0.4e1) * t384 + 96 * t632 * (L1 + 0.27e2 / 0.4e1) * t35 + 144 * t514) + 96 * (0.3e1 / 0.2e1 * t43 * t267 * t10 * t50 + L1 * t271 * t694) * L2) + t4 * (162 * t161 * t10 * t624 - 216 * t32 * (t43 * t625 * t308 + t209 / 4) * t362 + 72 * t31 * (t43 * (-t579 - t448 - t631 + t633 + t634) * t45 - 0.5e1 / 0.2e1 * t10 * L1 * (t256 + L2 * t715 - 0.6e1 / 0.5e1 * L1)) * L2 + 336 * t30 * (t43 * (t42 * (0.9e1 / 0.7e1 * t403 + 0.12e2 / 0.7e1 * t35 + 0.18e2 / 0.7e1 * t402) + L2 * (L1 * (0.9e1 / 0.14e2 * t403 + t35 + 0.9e1 / 0.7e1 * t402) + 0.36e2 / 0.7e1 * t402 + t736 + 0.18e2 / 0.7e1 * t403) + 0.15e2 / 0.7e1 * (0.6e1 / 0.5e1 * t402 + t35 + 0.3e1 / 0.5e1 * t403) * L1) * t308 + t258 / 14) * t42 + 48 * t43 * (3 * t378 + 2 * t756 + 9 * t42 * L1 * (t36 + 0.2e1 / 0.3e1 * t2) + 6 * t458 * t384 + t765 * t370) * t50 + 48 * t149 * t10 * L1 * t257) - 108 * t627 + 36 * t32 * (-0.9e1 / 0.2e1 * t669 * t778 * t50 - t579 - t448 - t631 + t633 + t634) * L2 + 60 * t31 * (-0.9e1 / 0.5e1 * t235 + (t643 + L2 * t647 * t71 * t645 + 0.12e2 / 0.5e1 * (t35 + 4 * t402 + 2 * t403) * L1) * L1) * t42 - 48 * t30 * L1 * (-0.3e1 / 0.2e1 * t43 * t592 * t10 * t50 + t579 + t580 + t583 + t586 + t587) - 48 * t192 * (L1 * t694 - t52) * L2) / 16;
        t857 = t362 * t1 / 2;
        t859 = 0.3e1 / 0.2e1 * q1__vel * t366;
        t860 = q1__vel * t369;
        t878 = t1 * L1;
        t894 = -t1;
        t896 = 12 * t377 * t894;
        res__2_4 = t338 * t606 * (24 * t5 * t43 * (q1__vel * t30 * t482 + t857 + t859 + t860) + t4 * (-36 * t43 * t9 * (t291 + 0.2e1 / 0.3e1 * L2) * L2 + 24 * (t31 * q1__vel * t482 + t30 * (t857 + t859 + t860) + ((q1__vel + 0.3e1 / 0.2e1 * q2__vel) * L2 + t878) * t82) * t45) + t43 * (-36 * t291 * ((q1__vel + q2__vel / 2) * L2 + 0.7e1 / 0.3e1 * q1__vel + q2__vel) * t42 + t896 + 36 * t362 * t894 - 36 * q1__vel * t381 - 12 * t72 * q1__vel * t365 - 24 * t860) + 24 * t9 * (-0.3e1 / 0.2e1 * t31 * t82 + t30 * t42 / 2 + t618) * t45) / 4;
        t926 = t4 * t342;
        res__2_5 = t202 * t359 * t606 * (-36 * t31 * t350 * t308 - 18 * t30 * (-0.2e1 / 0.3e1 * t45 * (t4 * t293 + q1__vel + q2__vel + q3__vel) * L2 + t43 * (t19 * t293 + t926) * L1) * L2 + 24 * t45 * L1 * (t4 * t38 * t293 + t149 * t9) - 12 * t43 * (t25 * t293 - t5 * t293 + t926) * t42) / 4;
        t964 = t359 * t606;
        res__2_6 = 6 * t202 * t964 * t9 * (-0.3e1 / 0.2e1 * t325 * t308 + t30 * (-0.3e1 / 0.2e1 * t168 * t82 + t45 * t42 / 2) - t4 * t44 + t149 * t308);
        t967 = t35 * L2;
        t971 = t2 * t42;
        t981 = L2 * t765;
        t982 = 6 * L1;
        t984 = L1 * (-t103 + t981 + t982);
        t987 = 24 * t32 * t984 * t35 * t42;
        t991 = t362 * t71 * t2;
        t993 = t42 * t675;
        t1000 = t38 * t45;
        t1003 = 3 + L1 + t148;
        t1015 = 6 * t240;
        t1016 = t991 + 6 * t993 - t1015;
        t1021 = 0.21e2 / 0.10e2 * t42;
        t1029 = t43 * t984 * t86;
        t1031 = 15 * q3__vel;
        t1034 = q2__vel * q3__vel;
        t1037 = q3__vel ^ 2;
        t1041 = 2 * q3__vel;
        t1045 = (q1__vel + q2__vel + 0.2e1 / 0.5e1 * q3__vel) * (q1__vel + q2__vel - t1041);
        t1058 = 2 * q2__vel;
        t1059 = q3__vel / 2;
        t1062 = t1034 / 2;
        t1063 = t1037 / 4;
        t1065 = (t35 + q1__vel * (t1058 - t1059) + t403 - t1062 - t1063) * L1;
        t1086 = 3 * t362;
        t1092 = L2 * (t365 + 9 * L1);
        t1093 = 3 * t365;
        t1118 = L1 + 9;
        t1120 = 8 * L1;
        t1133 = t377 * t2 * (L1 + 0.21e2 / 0.4e1);
        t1176 = 0.2e1 / 0.3e1 * q3__vel;
        t1188 = t35 * t106;
        t1195 = t458 + t73;
        t1201 = 3 * t495;
        t1203 = t377 * t1118 * t2;
        t1210 = 6 * L2 * t765 * t10;
        t1224 = 0.3e1 / 0.4e1 * t42 + t981 + t73;
        t1236 = t3 + 0.4e1 / 0.3e1 * t35 + (0.8e1 / 0.3e1 * q2__vel - 0.4e1 / 0.3e1 * q3__vel) * q1__vel + 0.4e1 / 0.3e1 * t403 - 0.4e1 / 0.3e1 * t1034 - 0.2e1 / 0.3e1 * t1037;
        t1254 = 3 * t240;
        t1265 = t1224 * L1;
        res__3_2 = t851 * t606 * (144 * t6 * (2 * L1 * t32 * t967 + t365 * t30 * t35 + 2 * t31 * t971 - t971) * t38 + t11 * (t987 - 288 * t31 * (t43 * t407 * t86 - t991 / 12 - t993 / 2 + t240 / 2) * L2 - 24 * t30 * (12 * t66 * t1000 + t1003 * t384) * t42 - 144 * t43 * t119 * L1 * t35 * t1000 - 12 * t1016 * L2) + t5 * (-180 * t32 * (t1021 + t72 + 0.16e2 / 0.5e1 * L1) * L1 * t967 - 24 * t31 * (t1029 + t42 * (t650 + q1__vel * (12 * q2__vel - t1031) - 15 * t1034 + 6 * t403 - 0.15e2 / 0.2e1 * t1037) + L2 * (5 * L1 * t1045 + 30 * t35 + (60 * q2__vel - 30 * q3__vel) * q1__vel + 30 * t403 - 30 * t1034 - 15 * t1037) + 24 * t1065) * t42 + t30 * (-24 * t43 * t1016 * t50 - 120 * (-0.9e1 / 0.10e2 * t362 + 0.9e1 / 0.5e1 * t42 * t505 + L2 * (t365 + 0.21e2 / 0.5e1 * L1) + 0.12e2 / 0.5e1 * t365) * t36) - 48 * (t43 * (t1086 + t42 * (0.15e2 / 0.2e1 + 0.13e2 / 0.4e1 * L1) + t1092 + t1093) * L1 * t86 - 0.5e1 / 0.4e1 * (t42 * (0.6e1 / 0.5e1 * t35 + (0.12e2 / 0.5e1 * q2__vel - 0.24e2 / 0.5e1 * q3__vel) * q1__vel + 0.6e1 / 0.5e1 * t403 - 0.24e2 / 0.5e1 * t1034 - 0.12e2 / 0.5e1 * t1037) + t72 * t1045 + 0.24e2 / 0.5e1 * t1065) * L2) * L2) + t4 * (18 * t32 * t36 * (t42 * t1118 + t1086 - t1120) * t42 + 180 * t31 * (t43 * (t1021 + t451 + t252) * L1 * t86 + t495 / 4 + 0.2e1 / 0.15e2 * t1133 + 0.2e1 / 0.15e2 * t362 * t104 * t2 + t42 * (-0.4e1 / 0.5e1 * t675 + 0.3e1 / 0.2e1 * t10) + 0.4e1 / 0.5e1 * L2 * t10 * t457 + 0.4e1 / 0.5e1 * t240) * L2 + 8 * t30 * (42 * t43 * t45 * (t42 * (0.39e2 / 0.28e2 * t35 + (0.39e2 / 0.14e2 * q2__vel + 0.6e1 / 0.7e1 * q3__vel) * q1__vel + 0.39e2 / 0.28e2 * t403 + 0.6e1 / 0.7e1 * t1034 + 0.3e1 / 0.7e1 * t1037) + L2 * (L1 * (t35 + q1__vel * (t1058 + 0.5e1 / 0.7e1 * q3__vel) + t403 + 0.5e1 / 0.7e1 * t1034 + 0.5e1 / 0.14e2 * t1037) + t736 + (0.48e2 / 0.7e1 * q2__vel + 0.12e2 / 0.7e1 * q3__vel) * q1__vel + 0.24e2 / 0.7e1 * t403 + 0.12e2 / 0.7e1 * t1034 + 0.6e1 / 0.7e1 * t1037) + 0.18e2 / 0.7e1 * L1 * (t35 + q1__vel * (t1058 + t1176) + t403 + 0.2e1 / 0.3e1 * t1034 + t1037 / 3)) + t1003 * L1 * t1188) * t42 + 48 * t43 * t618 * t35 * t1195 * t45 - 12 * (t1201 + t1203 + t676 + t42 * (-6 * t675 + 18 * t10) + t1210 + t1015) * L2) + 54 * t32 * t149 * t23 * L1 * t19 * t967 + 36 * t31 * (t43 * t23 * t1224 * L1 * t86 + 2 * t1236 * t74) * t42 + t30 * (48 * t43 * (0.3e1 / 0.4e1 * t495 + t1133 + 6 * t362 * t2 * t457 + t42 * (9 * t675 + 0.9e1 / 0.2e1 * t10) + 0.3e1 / 0.2e1 * L2 * t71 * t10 + t1254) * t50 + 24 * t149 * L1 * t19 * t1188) + 16 * t192 * (t43 * t1265 * t86 - 0.9e1 / 0.4e1 * t1236 * L2) * L2) / 16;
        t1279 = t363 + 2 * t971 + 6 * t10;
        t1287 = 0.18e2 / 0.7e1 * t993;
        t1289 = 0.30e2 / 0.7e1 * t240;
        t1305 = 0.3e1 / 0.2e1 * t378 + t756 + 3 * t993 + 9 * t209 + t1254;
        t1313 = t35 + q1__vel * (t1058 + q3__vel) + t403 + t1034 + t1037 / 2;
        t1316 = t161 * t19 * t1313 * t362;
        t1320 = t82 - t103 + t172;
        t1321 = t1320 * t36;
        t1389 = t1003 * t35;
        t1400 = L1 * t1313;
        t1433 = q2__vel * L1;
        t1438 = t403 * L1;
        t1440 = (q2__vel + q3__vel) ^ 2;
        res__3_3 = t851 * t606 * (t11 * (-72 * t161 * t1279 * t42 + t987 + 84 * t31 * (0.15e2 / 0.7e1 * t378 + t362 * t2 * (L1 + 0.36e2 / 0.7e1) + t1287 + 0.90e2 / 0.7e1 * t209 + t1289) * L2 - 48 * t30 * (t42 * (0.9e1 / 0.4e1 * L1 + 0.9e1 / 0.2e1) + t1092 + t1093) * L1 * t967 - 48 * t1305 * t267) + t5 * (-432 * t1316 + 72 * t32 * (t43 * t1279 * t50 + t1321) * L2 - 24 * t31 * (t1029 + t42 * (-0.105e3 / 0.2e1 * t35 + (-105 * q2__vel - 60 * q3__vel) * q1__vel - 60 * t1034 - 0.105e3 / 0.2e1 * t403 - 30 * t1037) + L2 * (t35 * (-22 * L1 - 105) + ((-44 * q2__vel - 26 * q3__vel) * L1 - 210 * q2__vel - 120 * q3__vel) * q1__vel + L1 * (-22 * t403 - 26 * t1034 - 13 * t1037) - 105 * t403 - 120 * t1034 - 60 * t1037) - 42 * L1 * (t35 + q1__vel * (t1058 + 0.8e1 / 0.7e1 * q3__vel) + 0.8e1 / 0.7e1 * t1034 + t403 + 0.4e1 / 0.7e1 * t1037)) * t42 + t30 * (-84 * t43 * (0.12e2 / 0.7e1 * t378 + t362 * (L1 + 0.30e2 / 0.7e1) * t2 + t1287 + 0.72e2 / 0.7e1 * t209 + t1289) * t50 - 96 * (t42 * (t73 + 0.9e1 / 0.4e1) + L2 * (t365 + 0.27e2 / 0.4e1 * L1) + 0.3e1 / 0.2e1 * t365) * t36) - 24 * (t268 * t1389 * t546 + 8 * t271 * (L2 * (0.7e1 / 0.4e1 * t35 + q1__vel * (0.7e1 / 0.2e1 * q2__vel + t1041) + 0.7e1 / 0.4e1 * t403 + 2 * t1034 + t1037) + t1400)) * L2) + t4 * (-27 * t161 * t1279 * t362 + 36 * t32 * (12 * t43 * t19 * t1313 * t45 + t1224 * t36) * t362 - 72 * t31 * (t43 * t1320 * t35 * t308 - 0.5e1 / 0.4e1 * t495 - 0.2e1 / 0.3e1 * t377 * (L1 + 0.33e2 / 0.8e1) * t2 - 0.4e1 / 0.3e1 * t362 * t497 + t42 * (t35 * (L1 - 0.15e2 / 0.2e1) + q1__vel * (2 * t1433 - 15 * q2__vel - t1031) + t1438 - 0.15e2 / 0.2e1 * t1440) - 0.5e1 / 0.2e1 * L2 * t10 * t715 + t1254) * L2 - 16 * t30 * (0.39e2 / 0.2e1 * t142 * L2 * (t42 * (0.30e2 / 0.13e2 * t35 + (0.60e2 / 0.13e2 * q2__vel + 0.24e2 / 0.13e2 * q3__vel) * q1__vel + 0.30e2 / 0.13e2 * t403 + 0.24e2 / 0.13e2 * t1034 + 0.12e2 / 0.13e2 * t1037) + L2 * (t35 * t55 + q1__vel * (L1 * (t1058 + 0.8e1 / 0.13e2 * q3__vel) + 0.120e3 / 0.13e2 * q2__vel + 0.48e2 / 0.13e2 * q3__vel) + L1 * (t403 + 0.8e1 / 0.13e2 * t1034 + 0.4e1 / 0.13e2 * t1037) + 0.60e2 / 0.13e2 * t403 + 0.48e2 / 0.13e2 * t1034 + 0.24e2 / 0.13e2 * t1037) + 0.24e2 / 0.13e2 * (t35 + q1__vel * (t1058 + t1059) + t403 + t1062 + t1063) * L1) + t1265 * t35 * t257) * L2 - 48 * t547 * t1389 * t546 - 16 * t1305 * t257) + 216 * t1316 - 36 * t32 * (-0.3e1 / 0.4e1 * t43 * t1279 * t45 * t778 * L2 + t1321) * L2 + 18 * t31 * (t43 * L1 * (t1086 + t42 * (13 + L1) + (16 + 0.16e2 / 0.3e1 * L1) * L2 + t1120) * t86 + t42 * (-35 * t35 + (-70 * q2__vel - 40 * q3__vel) * q1__vel - 35 * t403 - 40 * t1034 - 20 * t1037) - 0.44e2 / 0.3e1 * L2 * (t35 + q1__vel * (t1058 + 0.13e2 / 0.11e2 * q3__vel) + t403 + 0.13e2 / 0.11e2 * t1034 + 0.13e2 / 0.22e2 * t1037) * t71 - 40 * L1 * (t35 + q1__vel * (t1058 + 0.6e1 / 0.5e1 * q3__vel) + t403 + 0.6e1 / 0.5e1 * t1034 + 0.3e1 / 0.5e1 * t1037)) * t42 + t30 * (-12 * t142 * (t1201 + t1203 + t676 + t42 * (t35 * (-t982 + 18) + q1__vel * (-12 * t1433 + 36 * q2__vel + 36 * q3__vel) - 6 * t1438 + 18 * t1440) + t1210 + t1015) * L2 + 48 * t618 * t35 * t1195) - 8 * t192 * (t43 * t1389 * t546 + L2 * (-21 * t35 + (-42 * q2__vel - 24 * q3__vel) * q1__vel - 21 * t403 - 24 * t1034 - 12 * t1037) - 12 * t1400) * L2) / 16;
        t1592 = t362 * t1;
        t1594 = 2 * t42 * t1;
        t1595 = 6 * q1__vel;
        t1596 = 6 * q2__vel;
        t1597 = 6 * q3__vel;
        t1601 = 6 * t31 * (4 * t4 * t350 + t1592 + t1594 + t1595 + t1596 + t1597) * L2;
        t1619 = 48 * t4 * ((0.7e1 / 0.4e1 * q1__vel + 0.7e1 / 0.4e1 * q2__vel + q3__vel) * L2 + (q1__vel + q2__vel + t1059) * L1) * L2;
        t1622 = 8 * t362 * t754 * t1;
        t1624 = 24 * t42 * t878;
        t1627 = -72 * t9 * L2;
        t1629 = 24 * t9 * L1;
        t1642 = (4 * t5 * t350 + t4 * (t1592 + t1594 + t1595 + t1596 + t1597) + 3 * (t293 + 0.4e1 / 0.3e1 * q1__vel + 0.4e1 / 0.3e1 * q2__vel - t1176) * L2) * L2;
        res__3_4 = t359 * t606 * (t45 * (t1601 - 8 * t291 * q1__vel * (t4 * (t73 + 0.9e1 / 0.2e1 * L2) + t1224 * L2) - t1619 + t896 - t1622 - t1624 + t1627 - t1629) + 4 * t43 * (0.3e1 / 0.2e1 * t30 * t1642 + (t5 * (-t982 - 9 * L2) + t4 * t1003 * t42 + 3 * t149 * t19) * t313)) / 4;
        res__3_5 = t359 * t606 * (t45 * (t1601 - t1619 + t896 - t1622 - t1624 + t1627 - t1629) + 6 * t43 * t30 * t1642) / 4;
        res__3_6 = -6 * t964 * t9 * (-t31 * (t4 * L2 + 0.3e1 / 0.2e1) * t50 - t30 * (t354 - L2 / 2 + 0.3e1 / 0.2e1 * t4) * t117 + (t4 * t267 * L2 + L1 + t148) * t45);
        A = zeros(6, 6);
        A(1, 2) = res__1_2;
        A(1, 3) = res__1_3;
        A(1, 4) = res__1_4;
        A(1, 5) = res__1_5;
        A(1, 6) = res__1_6;
        A(2, 2) = res__2_2;
        A(2, 3) = res__2_3;
        A(2, 4) = res__2_4;
        A(2, 5) = res__2_5;
        A(2, 6) = res__2_6;
        A(3, 2) = res__3_2;
        A(3, 3) = res__3_3;
        A(3, 4) = res__3_4;
        A(3, 5) = res__3_5;
        A(3, 6) = res__3_6;
        A(1:3, 1:3) = eye(3);
        A(1:3, 4:6) = self.dt * eye(3);
    end


    function B = jacobian_dynamic_inputs(self, estimated)
        if estimated  
            q = self.q_est;
            dq = self.dq_est;
        else 
            q = self.q_true;
            dq = self.dq_true;
        end
        q1 = q(1);
        q2 = q(2);
        q3 = q(3);
        q1__vel = dq(1);
        q2__vel = dq(2);
        q3__vel = dq(3);
        L1 = self.L1;
        L2 = self.L2;


        B = zeros(6, 3);
        M = self.mass_matrix(estimated);
        B(4:6, :) = inverse(M);
    end
end % methods

end % Manipulator class
