function plot_measurement(map, time)
    z = map.z_hist{time};
    R = map.R_hist{time};

    for i = 1:numel(z)/2
        zi = z(2*i-1:2*i);
        Ri = R(2*i-1:2*i, 2*i-1:2*i);
        [data_x, data_y] = uncertainty_ellipsoid(zi, Ri);
        plot(data_x, data_y, "r:", "LineWidth", 1);
    end
end
