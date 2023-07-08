function knowledge_plot(t_axis, map_knowledge)
    plot(t_axis, map_knowledge(:, 1, 2));
    hold on;
    plot(t_axis, map_knowledge(:, 1, 1));
end
