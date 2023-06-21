function [Fi_new, ai_new] = linear_consensus(Fi, ai, Q)
    n_robots = numel(Fi);
    Fi_new = cell(n_robots, 1);
    ai_new = cell(n_robots, 1);

    for i = 1:n_robots
        Fi_new{i} = zeros(size(Fi{i}));
        ai_new{i} = zeros(size(ai{i}));
        for j = 1:n_robots
            Fi_new{i} = Fi_new{i} + Q(i,j)*Fi{j};
            ai_new{i} = ai_new{i} + Q(i,j)*ai{j};
        end
    end
end

