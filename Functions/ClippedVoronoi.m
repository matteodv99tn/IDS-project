function [V, R] = ClippedVoronoi(P)

    if size(P, 2) ~= 2
        error('P must be a Nx2 matrix');
    end

    [vx, vy] = voronoi(P(:,1), P(:,2));
    DT = delaunayTriangulation(P);
    [Vo, Ro] = voronoiDiagram(DT);


    V = Vo(2:end, :);
    allpoints = [vx(1, :), vx(2, :);
                 vy(1, :), vy(2, :)]';

    for i = 1:size(allpoints, 1)
        if ~any(ismember(V, allpoints(i, :), 'rows'))
            V = [V; allpoints(i, :)];
        end
    end

    N_original = size(Vo, 1);
    N_total = size(V, 1);
    N_new = N_total - N_original;

    R = cell(size(Ro));

    for i = 1:numel(R)
        R{i} = Ro{i} - 1;

        if R{i}(1) == 0
            idx_left  = find_childs(R{i}(2));
            idx_right = find_childs(R{i}(end));

            if length(R{i}) == 2
                R{i}(1) = idx_left(1);
                R{i}(end+1) = idx_left(2);
            else
                if length(idx_left) == 1
                    R{i}(1) = idx_left;
                else
                    candidates = V(idx_left, :);
                    delta = candidates - P(i, :);
                    dist = sqrt(delta(:, 1).^2 + delta(:, 2).^2);
                    [~, idx] = min(dist);
                    R{i}(1) = idx_left(idx);
                end

                if length(idx_right) == 1
                    R{i}(end+1) = idx_right;
                else
                    candidates = V(idx_right, :);
                    delta = candidates - P(i, :);
                    dist = sqrt(delta(:, 1).^2 + delta(:, 2).^2);
                    [~, idx] = min(dist);
                    R{i}(end+1) = idx_right(idx);
                end
            end
        end
    end


    function childs = find_childs(idx)
        parent = V(idx, :);
        all_childs = [];

        for j = 1:size(vx, 2)
            child1 = [vx(1, j), vy(1, j)];
            child2 = [vx(2, j), vy(2, j)];

            if isequal(child1, parent)
                idx = find(ismember(V, child2, 'rows'));
                all_childs(end+1) = idx;
            elseif isequal(child2, parent)
                idx = find(ismember(V, child1, 'rows'));
                all_childs(end+1) = idx;
            end
        end

        valid_childs = find(all_childs >= N_original);
        % if size(valid_childs, 1) ~= 1 || size(valid_childs, 2) ~= 1
        %     error('Something went wrong');
        % end

        childs = all_childs(valid_childs);
    end % find_childs

end
