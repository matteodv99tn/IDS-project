function S = cartesian_covariance_from_polar(z, P)
    H = [cos(z(2)) -z(1)*sin(z(2));
         sin(z(2))  z(1)*cos(z(2))];
    S = H*P*H';
end
