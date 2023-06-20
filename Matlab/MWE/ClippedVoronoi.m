function [V, R] = ClippedVoronoi(P)

    if size(P, 2) ~= 2
        error('P must be a Nx2 matrix');
    end

    voronoi(P(:,1), P(:,2));
    [vx, vy] = voronoi(P(:,1), P(:,2));
    DT = delaunayTriangulation(P);
    [Vo, Ro] = voronoiDiagram(DT);


    V = Vo(2:end, :);

    allpoints = [vx(1, :), vx(2, :); vy(1, :), vy(2, :)]';
    for i = 1:size(allpoints, 1)
        if ~any(ismember(V, allpoints(i, :), 'rows'))
            V(end+1, :) = allpoints(i, :);
        end
    end

    N_original = size(Vo, 1);
    N_total = size(V, 1);
    N_new = N_total - N_original;

    R = cell(size(Ro));

    for i = 1:numel(R)
        R{i} = Ro{i} - 1;

        if R{i}(1) == 0

            idx_left_all = find_childs(Ro{i}(2));
            idx_right_all = find_childs(Ro{i}(end));

            pt = P(i, :);

            left_idx = find_best_candidate(idx_left_all, pt);
            right_idx = find_best_candidate(idx_right_all, pt);

            R{i}(1) = left_idx;
            R{i}(end+1) = right_idx;
        end
    end



    function idx = find_best_candidate(idx_all, pt)
        distances = zeros(size(idx_all), 1);

        for j = 1:size(idx_all)
            distances(j) = norm(V(idx_all(j), :) - pt);
        end

        [~, min_idx] = min(distances);
        idx = idx_all(min_idx);
    end % find_best_candidate


    function childs = find_childs(idx)
        childs = [];
        parent = V(idx, :);
        fprintf(" --- %f %f\n", parent(1), parent(2));

        for j = 1:size(vx, 2)
            candidate1 = [vx(1, j), vy(1, j)];
            candidate2 = [vx(2, j), vy(2, j)];

            fprintf("%d - %d %d\n", j, isequal(parent, candidate1), isequal(parent, candidate2));

            if isequal(parent, candidate1)
                [row, col] = find(V == candidate2);
                childs(end+1) = row(1);
            elseif isequal(parent, candidate2)
                [row, col] = find(V == candidate1);
                childs(end+1) = row(1);
            end
        end
        childs = childs(childs > N_original);
    end % find_childs

end
