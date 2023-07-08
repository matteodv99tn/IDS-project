function detection_plot(t_axis, detection_hist, ref_idx)
    correct_classification = detection_hist == ref_idx;
    plot(t_axis, correct_classification(:, 1));
    ylim([-0.1 1.1]);
    yticks([0 1]);
    yticklabels(["wrong", "correct"]);
    grid on;
end

