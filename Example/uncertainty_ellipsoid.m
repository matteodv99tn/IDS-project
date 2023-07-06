function [x, y] = uncertainty_ellipsoid(mu, C, conf, n)
    % Draws the uncerainty ellipsoid of a 2D gaussian distribution with mean value mu and 
    % covariance matrix  C.
    % The ellipsoid is drawn with a confidence of conf and n points are used to draw it.
    % By default conf = 99% and n = 100.

    if nargin < 3
        conf = 0.99;
    end
    if nargin < 4
        n = 100;
    end
    if conf > 1
        conf = conf / 100;
    end

    r       = norminv((1-conf) / 2);
    theta   = linspace(0, 2*pi, n);
    circle  = r * [ cos(theta); ...
                    sin(theta) ];
    [eig_vectors, eig_values] = eig(C);
    
    data    = eig_vectors * sqrt(eig_values) * circle;
    x       = data(1, :) + mu(1);
    y       = data(2, :) + mu(2);
end
