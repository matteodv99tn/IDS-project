
function d = cast_ray(P1, P2, O, v)
    % Casts a ray starting from the point O and direction v and finds the intersection with the segment of vertices P1
    % and P2. 
    % Returns: 
    %   - d:    the distance of between O and P1P2 (in the direction of v)
    %   - t:    a parameter to identify if the casted ray is inside 

    global d_max;

    A = [P2-P1, -v];
    b = [O - P1];
    
    if abs(det(A)) < 1e-6
        d = d_max;
        return;
    end

    x = linsolve(A, b);
    t = x(1);
    d = x(2);

    valid_distance = (d > 0) & (d < d_max) & (t >= 0) & (t <= 1);

    d = valid_distance.*d + ~valid_distance.*d_max;

end
