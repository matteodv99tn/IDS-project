% close all;
clear;
clc;

setup;


%% --- Simulation setup
man     = Manipulator(5,5);         % initialize the manipulator and the camera     
cam     = Camera(60*pi/180, 101);
man.q   = [pi/2; -pi/2; -pi/2];     % set the init. cond. of the robot (avoiding singular confs.)

t = 0:dt:22;                        % initialize the vector of times

RF      = translation_matrix(3, 0); % determine the reference frame where to place the square
obj     = Square(RF);               % instantiate a square
scan    = Scan(man, cam, obj);      % create the scan given the initial configuration


%% --- Plots of the initial configuration
figure(2);  % The robot pose and the object
plot(man);
plot(obj);
axis equal

figure(3);  % The computed laserscan
plot(scan);
axis equal;
close all;


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


for k = 1:length(t)

    man.update_control_law();
    q_traj(:, k) = man.q;

    if mod(k, 150) == 0
        figure(1), clf;
        subplot(1, 2, 1);
        plot(man);
        plot(obj);
        axis equal;
        title(num2str(t(k)));
        xlim([-1 10]);
        ylim([-7, 10]);
        subplot(1, 2, 2);
        scan    = Scan(man, cam, obj);      % create the scan given the initial configuration
        plot(scan);
        axis equal;
        xlim([-1 30]);
        ylim([-15, 15]);
    end

    % if k == round(length(t)/2)
    %     man.controller.set_target([4; 2; -pi/2]);
    % end

end

figure; % joint behavior
plot(t, q_traj);
% yline(q_des);
