clear;
clc;
setup;


%% --- Setup 
t = 0:dt:10;

origin1 = [0; 0];
origin2 = [7; 0];
q0_man1 = [160; -130; -30]*pi/180;
man1 = Manipulator(3, 2, origin1);
man2 = Manipulator(2, 2, origin2);
man1.set_initial_joint_config(q0_man1)
man2.set_initial_joint_config(2*pi*rand(3, 1))

plot(man1);
plot(man2);


%% --- Reach default target
man1.set_controller(CartesianVelocityController());
man2.set_controller(CartesianPointController());
% man1.controller.enqueue_target([0; 3; 0], 8);
man2.controller.enqueue_target([7; 3; pi], 8);

man1.controller.set_target(1*[1, -1, 0]);





for k = 1:length(t)
    man1.update_control_law();
    man2.update_control_law();

    if mod(k, 50) == 0
        figure(1), clf;
        plot(man1);
        plot(man2);
        xlim([-5, 12]);
        ylim([-3, 12]);
        axis equal
        title(sprintf('t = %.2f', t(k)));
    end
end
