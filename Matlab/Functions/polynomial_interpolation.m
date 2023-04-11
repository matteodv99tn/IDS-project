function pars = polynomial_interpolation(TT, x_i, x_i_dot, x_f)
    % Performs a fitting with a polynomial of order 6
    res__1_1 = x_i;
    res__1_2 = x_i_dot;
    t1 = TT * x_i_dot;
    t6 = TT ^ 2;
    res__1_4 = 0.1e1 / t6 / TT * (-6 * t1 + 10 * x_f - 10 * x_i);
    t13 = t6 ^ 2;
    res__1_5 = 0.1e1 / t13 * (8 * t1 - 15 * x_f + 15 * x_i);
    res__1_6 = 0.1e1 / t13 / TT * (-3 * t1 + 6 * x_f - 6 * x_i);
    pars = zeros(length(x_i), 6);
    pars(:, 1) = res__1_1;
    pars(:, 2) = res__1_2;
    pars(:, 4) = res__1_4;
    pars(:, 5) = res__1_5;
    pars(:, 6) = res__1_6;
end
