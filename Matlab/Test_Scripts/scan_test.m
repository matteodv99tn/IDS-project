setup;
close all;
clc;
% configuration_name = "tests";

obj     = dataset{randi(length(dataset))};
camera  = Camera();
n_scans = 10;

figure(1);
for k = 1:n_scans 
    
    theta   = 2*pi*rand(1);
    r       = 2 + 1.5*rand(1);
    y       = 0.5 * randn(1);
    delta   = randn(1) * 10 * pi/180;
    
    RF      = rotation_matrix(theta) ...
              * translation_matrix(-r, y) ...
              * rotation_matrix(delta);
    scan    = Scan(RF, camera, obj);
    [seeds, feat, n] = extract_features(scan);

    tab = uitab("title", sprintf("Scan %d", k));
    axes(tab), hold on;
    plot(scan);
    for i = 1:length(seeds)
        plot(seeds{i});
    end
    plot(feat(1,:), feat(2,:), 'r*');

    axis equal;
    grid on;
end





