function plot_map(map, time)
    if isempty(map.x_hist{time}) && time > 1
        time = time - 1;
    end
    x = map.x_hist{time};
    P = map.P_hist{time};

    for i = 1:numel(x)/2
        xi = x(2*i-1:2*i);
        Pi = P(2*i-1:2*i, 2*i-1:2*i);
        [data_x, data_y] = uncertainty_ellipsoid(xi, Pi);
        plot(data_x, data_y, "b", "LineWidth", 1);
    end
end
