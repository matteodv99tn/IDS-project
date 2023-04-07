
function trasl = translation_matrix(X, Y) 

    if nargin == 1
        x = X(1);
        y = X(2);
    else
        x = X;
        y = Y;
    end

    trasl = [ 1, 0, x; ...
              0, 1, y; ...
              0, 0, 1
              ];

end
