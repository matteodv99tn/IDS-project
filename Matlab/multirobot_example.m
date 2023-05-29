clear;
clc;
setup;


%% --- Setup 
t = 0:dt:10;

origin1 = [0; 0];
origin2 = [7; 0];
q0_man1 = [160; -130; -30]*pi/180;
man1 = Manipulator(3, 2, origin1);
man2 = Manipulator(2, 3, origin2);
man1.set_initial_joint_config(q0_man1)
man2.set_initial_joint_config([0; 1; -1]) % 2*pi*rand(3, 1))

plot(man1);
plot(man2);


%% --- Reach default target
man1.set_controller(CartesianVelocityController());
man2.set_controller(CartesianPointController());
% man1.controller.enqueue_target([0; 3; 0], 8);
man2.controller.enqueue_target([7; 3; pi], 5);

man1.controller.set_target(0.0*[1, 0, 0]);





for k = 1:length(t)
    man1.update_kinematic_dynamics();
    man2.update_kinematic_dynamics();
    if mod(k, 150) == 0
        figure(1), clf;
        plot(man1);
        plot(man2);
        xlim([-5, 12]);
        ylim([-3, 12]);
        axis equal
        title(sprintf('t = %.2f', t(k)));
    end
end


figure(2), clf, hold on;
plot(man2.error(1,:));
plot(3*man2.sigma(1,:));
plot(-3*man2.sigma(1,:));

figure(3), clf, hold on;
plot(man2.error(3,:));
plot(3*man2.sigma(3,:));
plot(-3*man2.sigma(3,:));

figure(4), clf, hold on;
plot(man2.d_error(3,:));
plot(3*man2.d_sigma(3,:));
plot(-3*man2.d_sigma(3,:));

figure(5), clf, hold on;
plot(man2.d_error(1,:));
plot(3*man2.d_sigma(1,:));
plot(-3*man2.d_sigma(1,:));
