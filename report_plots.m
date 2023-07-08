close all;
clear;
clc;

addpath("Functions/plot_utils");

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


function set_plot(fig_height)
    set(0, "DefaultAxesFontSize", 8);
    set(0, "DefaultFigureColor", "w");
    set(0, "defaulttextinterpreter", "tex");
    set(0, "DefaultAxesFontName", "times");
    set(gcf, "Units", "centimeters");
    set(gcf, "Position", [0 0 8.6 fig_height]);
    box on;
    grid on;
end
