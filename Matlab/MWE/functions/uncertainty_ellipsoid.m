function [x, y] = uncertainty_ellipsoid(mu, S)
    theta   = linspace(0, 2*pi, 101);
    circle  = 3 * [cos(theta); sin(theta)];
    [e_vect, e_vals] = eig(S);
    data    = e_vect * sqrt(e_vals) * circle;
    x       = data(1, :) + mu(1);
    y       = data(2, :) + mu(2);
end
