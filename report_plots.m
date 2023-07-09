close all;
clear;
clc;

setup;


file1 = "Results/test_1a/simulation_001.mat";
file2 = "Results/test_2a/simulation_001.mat";
file3 = "Results/test_3a/simulation_001.mat";

t_axis = 0:config.simulation.dt*config.simulation.k_meas:config.simulation.max_t;

if ~exist("Results/Figures/Report", "dir")
    mkdir("Results/Figures/Report");
end

%  ____  _       _     _
% |  _ \| | ___ | |_  / |
% | |_) | |/ _ \| __| | |
% |  __/| | (_) | |_  | |
% |_|   |_|\___/ \__| |_|
%
figure(1), clf, hold on;
load(file1);
title("Detection algorithm");
detection_plot(t_axis, detect_hist, obj_i);
set_plot(1.9);
set(gca,'xticklabel',[]);
exportgraphics(gcf, "Results/Figures/Report/detection_a.eps");

figure(2), clf, hold on;
load(file2);
detection_plot(t_axis, detect_hist, obj_i);
set_plot(1.5);
set(gca,'xticklabel',[]);
exportgraphics(gcf, "Results/Figures/Report/detection_b.eps");

figure(3), clf, hold on;
load(file3);
detection_plot(t_axis, detect_hist, obj_i);
set_plot(2.4);
xlabel("Time [s]");
labels = get(gca, "xticklabel");
labels{end} = [];
set(gca, "xticklabel", labels);
exportgraphics(gcf, "Results/Figures/Report/detection_c.eps");


%   ____
%  / ___|___  _ ____   _____ _ __ __ _  ___ _ __   ___ ___
% | |   / _ \| '_ \ \ / / _ \ '__/ _` |/ _ \ '_ \ / __/ _ \
% | |__| (_) | | | \ V /  __/ | | (_| |  __/ | | | (_|  __/
%  \____\___/|_| |_|\_/ \___|_|  \__, |\___|_| |_|\___\___|
%                                |___/
close all;
file1 = "Results/test_hexa_1a/simulation_005.mat";
file2 = "Results/test_hexa_2a/simulation_001.mat";
file3 = "Results/test_hexa_3a/simulation_004.mat";

figure(), clf, hold on;
title("Discovered vertices");
load(file1);
knowledge_plot(t_axis, map_knowledge);
ylim([0, 7]);
yticks([0, 2, 4, 6]);
set_plot(2.4);
set(gca,'xticklabel',[]);
exportgraphics(gcf, "Results/Figures/Report/convergence_a.eps");

figure(), clf, hold on;
load(file2);
knowledge_plot(t_axis, map_knowledge);
ylim([0, 7]);
yticks([0, 2, 4, 6]);
set_plot(2);
set(gca,'xticklabel',[]);
exportgraphics(gcf, "Results/Figures/Report/convergence_b.eps");

figure(), clf, hold on;
load(file3);
knowledge_plot(t_axis, map_knowledge);
ylim([0, 7]);
yticks([0, 2, 4, 6]);
set_plot(2.6);
xlabel("Time [s]");
labels = get(gca, "xticklabel");
labels{end} = [];
set(gca, "xticklabel", labels);
exportgraphics(gcf, "Results/Figures/Report/convergence_c.eps");




close all;
file1 = "Results/test_star_1a/simulation_001.mat";
file2 = "Results/test_star_2a/simulation_001.mat";
file3 = "Results/test_star_3a/simulation_002.mat";

figure(), clf, hold on;
title("Discovered vertices");
load(file1);
knowledge_plot(t_axis, map_knowledge);
ylim([0, 12]);
yticks(0:3:12);
set_plot(2.2);
set(gca,'xticklabel',[]);
exportgraphics(gcf, "Results/Figures/Report/convergence_a2.eps");

figure(), clf, hold on;
load(file2);
knowledge_plot(t_axis, map_knowledge);
ylim([0, 12]);
yticks(0:3:12);
set_plot(1.8);
set(gca,'xticklabel',[]);
exportgraphics(gcf, "Results/Figures/Report/convergence_b2.eps");

figure(), clf, hold on;
load(file3);
knowledge_plot(t_axis, map_knowledge);
ylim([0, 12]);
yticks(0:3:12);
set_plot(2.6);
xlabel("Time [s]");
labels = get(gca, "xticklabel");
labels{end} = [];
set(gca, "xticklabel", labels);
exportgraphics(gcf, "Results/Figures/Report/convergence_c2.eps");


file2 = "Results/test_star_2a_v2/simulation_001.mat";
figure(), clf, hold on;
title("Discovered vertices");
load(file2);
knowledge_plot(t_axis, map_knowledge);
ylim([0, 12]);
yticks(0:3:12);
set_plot(2.8);
xlabel("Time [s]");
labels = get(gca, "xticklabel");
labels{end} = [];
set(gca, "xticklabel", labels);
exportgraphics(gcf, "Results/Figures/Report/convergence_comparison.eps");
%  ____  _               ____  _       _
% / ___|| |_ __ _ _ __  |  _ \| | ___ | |_ ___
% \___ \| __/ _` | '__| | |_) | |/ _ \| __/ __|
%  ___) | || (_| | |    |  __/| | (_) | |_\__ \
% |____/ \__\__,_|_|    |_|   |_|\___/ \__|___/
%
close all;
fprintf(" --- Star test \n");
file1 = "Results/test_star_1a/simulation_001.mat";
file2 = "Results/test_star_1c/simulation_001.mat";
file3 = "Results/test_star_3a/simulation_001.mat";
file4 = "Results/test_star_3c/simulation_002.mat";

figure(), clf, hold on;
load(file1);
title("Detection algorithm on a star object");
detection_plot(t_axis, detect_hist, obj_i);
set_plot(1.7);
set(gca,'xticklabel',[]);
exportgraphics(gcf, "Results/Figures/Report/star_a.eps");
fprintf("> 1a) estimated pose: %.3f, %.3f, %.3f\n", params(1), params(2), params(3));

figure(), clf, hold on;
load(file2);
detection_plot(t_axis, detect_hist, obj_i);
set_plot(2.2);
xlabel("Time [s]");
labels = get(gca, "xticklabel");
labels{end} = [];
set(gca, "xticklabel", labels);
exportgraphics(gcf, "Results/Figures/Report/star_b.eps");
fprintf("> 1c) estimated pose: %.3f, %.3f, %.3f\n", params(1), params(2), params(3));




figure(), clf, hold on;
load(file3);
title("Detection algorithm on a star object");
detection_plot(t_axis, detect_hist, obj_i);
set_plot(1.7);
set(gca,'xticklabel',[]);
exportgraphics(gcf, "Results/Figures/Report/star_c.eps");
fprintf("> 3a) estimated pose: %.3f, %.3f, %.3f\n", params(1), params(2), params(3));

figure(), clf, hold on;
load(file4);
detection_plot(t_axis, detect_hist, obj_i);
set_plot(2.2);
xlabel("Time [s]");
labels = get(gca, "xticklabel");
labels{end} = [];
set(gca, "xticklabel", labels);
exportgraphics(gcf, "Results/Figures/Report/star_d.eps");
fprintf("> 3c) estimated pose: %.3f, %.3f, %.3f\n", params(1), params(2), params(3));


%  _   _                  ____  _       _
% | | | | _____  ____ _  |  _ \| | ___ | |_ ___
% | |_| |/ _ \ \/ / _` | | |_) | |/ _ \| __/ __|
% |  _  |  __/>  < (_| | |  __/| | (_) | |_\__ \
% |_| |_|\___/_/\_\__,_| |_|   |_|\___/ \__|___/
%
close all;
fprintf(" --- Hexa test \n");
file1 = "Results/test_hexa_1a/simulation_004.mat";
file2 = "Results/test_hexa_1c/simulation_006.mat";
file3 = "Results/test_hexa_3a/simulation_004.mat";
file4 = "Results/test_hexa_3c/simulation_004.mat";

figure(), clf, hold on;
load(file1);
title("Detection algorithm on a star object");
detection_plot(t_axis, detect_hist, obj_i);
set_plot(1.9);
set(gca,'xticklabel',[]);
exportgraphics(gcf, "Results/Figures/Report/hexa_a.eps");
fprintf("> 1a) estimated pose: %.3f, %.3f, %.3f\n", params(1), params(2), params(3));

figure(), clf, hold on;
load(file2);
detection_plot(t_axis, detect_hist, obj_i);
set_plot(1.5);
set(gca,'xticklabel',[]);
exportgraphics(gcf, "Results/Figures/Report/hexa_b.eps");
fprintf("> 1c) estimated pose: %.3f, %.3f, %.3f\n", params(1), params(2), params(3));

figure(), clf, hold on;
load(file3);
detection_plot(t_axis, detect_hist, obj_i);
set_plot(1.5);
set(gca,'xticklabel',[]);
exportgraphics(gcf, "Results/Figures/Report/hexa_c.eps");
fprintf("> 3a) estimated pose: %.3f, %.3f, %.3f\n", params(1), params(2), params(3));

figure(), clf, hold on;
load(file4);
detection_plot(t_axis, detect_hist, obj_i);
set_plot(2.4);
xlabel("Time [s]");
labels = get(gca, "xticklabel");
labels{end} = [];
set(gca, "xticklabel", labels);
exportgraphics(gcf, "Results/Figures/Report/hexa_d.eps");
fprintf("> 3c) estimated pose: %.3f, %.3f, %.3f\n", params(1), params(2), params(3));



close all;

%  _____                 _   _
% |  ___|   _ _ __   ___| |_(_) ___  _ __  ___
% | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
% |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
% |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
%
function set_plot(fig_height)
    set(0, "DefaultAxesFontSize", 8);
    set(0, "DefaultFigureColor", "w");
    set(0, "defaulttextinterpreter", "tex");
    set(0, "DefaultAxesFontName", "times");
    set(gcf, "Units", "centimeters");
    set(gcf, "Position", [0 0 8.89 fig_height]);
    box on;
    grid on;
end
