% close all;
clear;
clc;

setup;


%% --- Simulation setup
man     = Manipulator(5,5);        % initialize the manipulator and the camera     
cam     = Camera(60*pi/180, 201);

man.q_true  = [pi/2; -pi/2; -pi/2];     % set the init. cond. of the robot (avoiding singular confs.)
man.q  = [pi/2; -pi/2; -pi/2];     % set the init. cond. of the robot (avoiding singular confs.)

t = 0:dt:22;                        % initialize the vector of times

RF      = translation_matrix(3, 0); % determine the reference frame where to place the square
% obj     = dataset{randi(length(dataset))};               
% obj.RF  = RF;
obj     = Square(RF);               % instantiate a square
scan    = Scan(man, cam, obj);      % create the scan given the initial configuration


%% --- Dynamic simulation
% In this case we use a cartesian point controller, i.e. we specify a required configuration in 
% cartesian space and the manipulator will achieve it.
% We can use JointPointController that instead controls the joint coordinate directly.
% At mid simulation, the target is also changed.

man.set_controller(CartesianPointController());
man.controller.set_target([2; 0; 0]);
man.controller.enqueue_target([3; 7; -pi/2]);
man.controller.enqueue_target([3; 3; -pi/2]);
man.controller.enqueue_target([0; 2; -pi/4]);
man.controller.enqueue_target([1; -3; pi/3]);
man.controller.enqueue_target([6; -2; 0.8*pi]);
q_traj = zeros(3, length(t));


map = MapEstimator();

for k = 1:length(t)

    man.update_control_law();
    q_traj(:, k) = man.q;

    if mod(k, 150) == 0
        figure(1), clf;
        plot(man);
        plot(obj);
        plot(cam, man.EE_frame());
        title(num2str(t(k)));
        xlim([-1 10]);
        ylim([-7, 10]);
        axis equal;
    end
    if mod(k, 150) == 0
        figure(2), clf, hold on;
        scan = Scan(man, cam, obj);      % create the scan given the initial configuration
        [seeds, feat, ~] = extract_features(scan);
        plot(scan);
        for j = 1:length(seeds) 
            plot(seeds{j});
        end
        if ~isempty(feat)
            plot(feat(1,:), feat(2, :), "*r");
        end
        xlim([-1 30]);
        ylim([-15, 15]);
        grid on;
        axis equal;

        map.KF_update_step(man, scan, cam);

        figure(3), clf, hold on;
        plot(obj);
        plot(map);
        for i = 1:size(feat, 2)
            [z, R] = project_features(man, cam, feat(:, i));
            [x, y] = uncertainty_ellipsoid(z, R);
            plot(x, y, "r");
        end
        xlim([1 6]);
        ylim([-2, 2]);
        grid  on;
        axis equal;
    end
end

figure; % joint behavior
plot(t, q_traj);
% yline(q_des);
