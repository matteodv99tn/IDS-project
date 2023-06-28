% close all;
clear;
clc;

setup;


obj = RegularPolygon(5);
cam = Camera();

test_positions = {
    rototranslation_matrix(-2, 0, 0),
    rototranslation_matrix(-2, 0.5, 0),
    rototranslation_matrix(-1.2, 1.5, -0.7),
    rototranslation_matrix(-0.9, 0.6, 0.1),
    rototranslation_matrix(-0.7, -3, pi/2),
    };
scans = cellfun(@(pos) Scan(pos, cam, obj), test_positions, "UniformOutput", false);


figure(100), clf;

for i = 1:length(test_positions)

    figure(100), hold on;
    plot(obj);
    origin = test_positions{i} * [0; 0; 1];
    delta = test_positions{i} * [1; 0; 0];
    quiver(origin(1), origin(2), delta(1), delta(2));
    axis equal;
    xlim([-5, 5]);
    ylim([-5, 5]);


    scan = scans{i};

    figure(i), clf, hold on;
    plot(scan);
    grid on;
    axis equal;

    [seeds, features, n_removed] = extract_features(scan);
    cellfun(@plot, seeds);
    plot(features(1, :), features(2, :), "r*");

    xlim([-1, 10]);
    ylim([-6, 6]);
    grid on;

end



