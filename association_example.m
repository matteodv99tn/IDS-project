clear;
clc;

setup;


obj = dataset{1};

x_obj = 2; % -2+4*rand();
y_obj = 0; % -2+4*rand();
theta_obj = 0;
obj.RF = rototranslation_matrix(x_obj, y_obj, theta_obj);
points = obj.get_projected_polygon();

N_points = size(points, 2) - 1;
N_meas = ceil(N_points/2);

meas = points(1:2, randperm(N_points-1, N_meas));

R = cell(1, N_meas);
z = cell(1, N_meas);
xz_sum = 0;
yz_sum = 0;
for i = 1:N_meas
    R{i} = 0.02*randn(2);
    R{i} = R{i}*R{i}';
    z{i} = meas(:, i) + mvnrnd([0;0], R{i})';
    xz_sum = xz_sum + z{i}(1);
    yz_sum = yz_sum + z{i}(2);
end
xz_mean = xz_sum / N_meas;
yz_mean = yz_sum / N_meas;

figure(1), clf, hold on;
plot(obj)

for i = 1:N_meas
    [xd, yd] = uncertainty_ellipsoid(z{i}, R{i});
    plot(xd, yd, "r--");
    plot(z{i}(1), z{i}(2), 'r*');
end
axis equal;
grid on;


x_test = -3:0.5:3;
for i = 1:length(x_test)
    x_curr = x_test(i);
    params = [x_curr, 0, 0];
    c1 = find_cost(params, z, R, obj);
    c2 = find_cost(params, z, R, dataset{3});
    fprintf("%d) x = %f, c1 = %f, c2 = %f\n", i, x_curr, c1, c2);
end



opts = optimset( ...
    "Display", "off", ...
    "MaxIter", 1e5, ...
    "MaxFunEvals", 1e6, ...
    "TolFun", 1e-12, ...
    "TolX", 1e-12 ...
    );
lowerB = [-3, 3, -pi];
upperB = [3, 3, pi];

x0 = [xz_mean, yz_mean, 0];

x = fminsearch(...
    @(x) find_cost(x, z, R, obj), ...
    x0, opts ...
    );
c = find_cost(x, z, R, obj)
fprintf("Minimization over good obj: %f\n", c);


obj_optimized = obj;
obj_optimized.RF = rototranslation_matrix(x(1), x(2), x(3));
points = obj_optimized.get_projected_polygon();
plot(points(1, :), points(2, :), "g--");

x = fminsearch(...
    @(x) find_cost(x, z, R, dataset{3}), ...
    x0, opts ...
    );
c = find_cost(x, z, R, obj)
fprintf("Minimization over bad obj: %f\n", c);

% paremeters = [x, y, theta] for rototranslation
function c = find_cost(parameters, z, R, compare_object)

    obj = compare_object;
    obj.RF = rototranslation_matrix(parameters(1), parameters(2), parameters(3));
    points = obj.get_projected_polygon();
    points = points(1:2, 1:end-1);

    c = 0;
    for i = 1:length(z)
        delta = points - z{i};
        dist = vecnorm(delta);
        [~, min_idx] = min(dist);

        pt = points(:, min_idx);
        c = c + mahalanobis_distance(z{i}, pt, R{i});
    end

    c = c / length(z);
end
