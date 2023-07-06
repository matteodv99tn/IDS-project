function [z, R] = project_features(manipulator, camera, features)

    n_features = size(features, 2);
    z = zeros(2*n_features, 1);
    R = zeros(length(z), length(z));

    for i = 1:n_features 
        x = features(:, i);
        [h, H_q, H_x] = inverse_observation_model(manipulator, x);
        P = blkdiag(manipulator.R_q, compute_camera_covariance(camera, x));
        H = [H_q, H_x];
        z(2*i-1:2*i) = h;
        R(2*i-1:2*i, 2*i-1:2*i) = H * P * H';
    end
end


function R = compute_camera_covariance(camera, z)

    Q = camera.polar_covariance;
    polar   = cartesian_to_polar(z);
    r       = polar(1);
    theta   = polar(2);
    H       = [ cos(theta),     -r * sin(theta);
                sin(theta),      r * cos(theta)]; 
    R       = H * Q * H';
end
