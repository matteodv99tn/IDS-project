function polar_data = cartesian_to_polar(cartesian_data)
    % Function that converts a 2-D cartestian point (x, y) into the corresponding polar vector 
    % (r, theta).

    polar_data    = zeros(size(cartesian_data));
    polar_data(1) = sqrt(cartesian_data(1)^2 + cartesian_data(2)^2);
    polar_data(2) = atan2(cartesian_data(2), cartesian_data(1));
end
