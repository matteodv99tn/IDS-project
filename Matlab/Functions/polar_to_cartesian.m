function cartesian_data = polar_to_cartesian(polar_data)
    % This function converts a 2-D polar vector (d, theta) into a cartesian pair (x, y)
    cartesian_data    = zeros(size(polar_data));
    cartesian_data(1) = polar_data(1) * cos(polar_data(2));
    cartesian_data(2) = polar_data(1) * sin(polar_data(2));
end
