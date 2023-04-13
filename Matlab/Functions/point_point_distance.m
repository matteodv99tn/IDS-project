function dist = point_point_distance(point1, point2)
    % Given two points, it returns the Euclidean distance between them
    delta = point2 - point1;
    dist  = sqrt(transpose(delta) * delta);
end
