function plot_voronoi_cell(planner, time)
    if isempty(planner.allreg_hist{time})
        time = time - 1;
    end
    plot(planner.allreg_hist{time});
end
