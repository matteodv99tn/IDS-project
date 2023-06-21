function res = voronoi_join_cells(c1, c2)
    [pair, c2] = common_pair(c1, c2);

    p1 = c1(1:find(c1 == pair(1), 1, 'First'));
    p3 = c1(find(c1 == pair(2), 1, 'Last'):end);

    if c2(1) == pair(1)
        p2 = c2(2:end-1);
    else
        yy = [c2, c2];
        p2 = yy(find(yy==pair(1), 1, 'First')+1:find(yy==pair(2), 1, 'Last')-1);
    end

    res = [p1, p2, p3];


    function [pair, y_new] = common_pair(x, y)
        y_new = y;
        yy = [y, y];
        x = [x, x(1)];
        for i = 1:length(x)-1
            pair = x(i:i+1);
            for j = 1:length(y)
                compare = [yy(j+1), yy(j)];
                if isequal(pair, compare)
                    return
                end
            end
        end
        fprintf("Flipped\n")
        y_new = flip(y);
        pair = [y(1), y(end)];
    end
end







