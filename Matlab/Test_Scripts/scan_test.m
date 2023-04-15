setup;
clc;
% configuration_name = "tests";

square  = Square();
camera  = Camera(pi/3, 41);
RF      = rototranslation_matrix(-1.3, 1.3, -pi/4);
%RF      = rototranslation_matrix(-3, 0, 0);
scan    = Scan(RF, camera, square);
[seeds, feat, n] = extract_features(scan);

fprintf("Removed %d features\n", n);

figure(1), clf, hold on;
orig = RF*[0; 0; 1];
plot(square);
% plot(orig(1), orig(2), "+");
plot(camera, RF);
axis equal;
xlim([-3, 3]);
ylim([-3, 3]);

figure(2), clf, hold on;
plot(scan);
for i = 1:length(seeds)
    plot(seeds{i});
end
xlim([-3, 3]);
ylim([-3, 3]);
plot(feat(1, :), feat(2,:), "*r");
axis equal;


% figure(3), clf, hold on;
% plot(scan)
% s_orig = seeds{1}
% s_mod  = seeds{1};
% s_mod.grow(scan)
