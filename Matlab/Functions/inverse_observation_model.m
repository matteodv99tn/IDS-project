function [h, H_q, H_o] = inverse_observation_model(manipulator, observation)
    % Inverse observation model of the features, i.e. the estimated landmark location given the 
    % manipulator configuration and the observation (referred to the end-effector).
    % Furthermore, the function might return the jacobian of such function with respect to both 
    % robot and observation.
    %
    % All computations are generated by Maple

    q1    = manipulator.q_true(1);
    q2    = manipulator.q_true(2);
    q3    = manipulator.q_true(3);
    L1    = manipulator.L1;
    L2    = manipulator.L2;
    xo    = manipulator.origin(1);
    yo    = manipulator.origin(2);
    x_obs = observation(1);
    y_obs = observation(2);

    % Inverse observation equation
    t1 = cos(q3);
    t3 = sin(q3);
    t5 = x_obs * t1 - y_obs * t3 + L2;
    t6 = cos(q2);
    t10 = -y_obs * t1 - x_obs * t3;
    t11 = sin(q2);
    t13 = t11 * t10 + t6 * t5 + L1;
    t14 = cos(q1);
    t16 = sin(q1);
    t17 = -t10;
    t20 = t5 * t11;
    res__1_1 = -t6 * t17 * t16 + t14 * t13 - t16 * t20 + xo;
    res__2_1 = t16 * t13 + t14 * (t6 * t17 + t20) + yo;
    h = zeros(2, 1);
    h(1, 1) = res__1_1;
    h(2, 1) = res__2_1;


    if nargout == 1 
        return;
    end 

    % Jacobian w.r.t. the robot joint configuration
    t1 = cos(q3);
    t2 = x_obs * t1;
    t3 = sin(q3);
    t4 = y_obs * t3;
    t5 = -t2 + t4 - L2;
    t6 = cos(q2);
    t10 = y_obs * t1 + x_obs * t3;
    t11 = sin(q2);
    t12 = t11 * t10;
    t14 = sin(q1);
    t17 = -t5;
    t18 = t17 * t11;
    t19 = t6 * t10 + t18;
    t20 = cos(q1);
    res__1_1 = t14 * (t6 * t5 - L1 + t12) - t20 * t19;
    t22 = -t10;
    t23 = t6 * t22;
    t26 = t6 * t17;
    t27 = -t12 + t26;
    res__1_2 = t20 * (-t18 + t23) - t14 * t27;
    t29 = t4 - t2;
    t31 = t11 * t29 + t23;
    res__1_3 = t20 * t31 + (t6 * t29 + t12) * t14;
    t39 = t14 * t19;
    res__2_1 = t20 * (t11 * t22 + L1 + t26) - t39;
    res__2_2 = t20 * t27 - t39;
    res__2_3 = t20 * (-t6 * t29 - t12) + t14 * t31;
    H_q = zeros(2, 3);
    H_q(1, 1) = res__1_1;
    H_q(1, 2) = res__1_2;
    H_q(1, 3) = res__1_3;
    H_q(2, 1) = res__2_1;
    H_q(2, 2) = res__2_2;
    H_q(2, 3) = res__2_3;

    % Jacobian w.r.t. the observation vector   
    t1 = sin(q2);
    t2 = cos(q1);
    t4 = sin(q1);
    t5 = cos(q2);
    t7 = -t2 * t1 - t5 * t4;
    t8 = sin(q3);
    t12 = -t4 * t1 + t2 * t5;
    t13 = cos(q3);
    res__1_1 = t13 * t12 + t8 * t7;
    t18 = -t7 * t13;
    res__1_2 = -t12 * t8 - t18;
    res__2_1 = t12 * t8 + t18;
    res__2_2 = res__1_1;
    H_o = zeros(2, 2);
    H_o(1, 1) = res__1_1;
    H_o(1, 2) = res__1_2;
    H_o(2, 1) = res__2_1;
    H_o(2, 2) = res__2_2;
    
end
