function d = mahalanobis_distance(x, mu, S) 
    % Computes the mahalanobis distance of a point x considering a normal distribution of mean mu 
    % and covariance matrix S
    delta = x - mu;
    d     = sqrt(delta' * inv(S) * delta);
end
