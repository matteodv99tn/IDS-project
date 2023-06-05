clear;
clc;
setup;


%% --- Setup 
t = 0:dt:10;

origin1 = [-3; 0];
origin2 = [3; 1];
q0_man1 = [160; -130; -30]*pi/180;
man1 = Manipulator(2, 2, origin1);
man2 = Manipulator(2, 2, origin2);
man1.set_initial_joint_config(q0_man1)
man2.set_initial_joint_config([0; 1; -1]) % 2*pi*rand(3, 1))

obj = dataset{randi(numel(dataset))};
cam1 = Camera(60*pi/180, 101);
cam2 = Camera(45*pi/180, 101);

Q1 = [  1,      0;
        1/2,    1/2];
Q2 = [  2/3,    1/3;
        0,      1];

plot(man1);
plot(man2);


%% --- Reach default target
% man1.set_controller(CartesianVelocityController());
% man2.set_controller(CartesianVelocityController());
% man1.controller.set_target([0; 0; 0]);
% man2.controller.set_target([0; 0; 0]);
man1.set_controller(CartesianPointController());
man2.set_controller(CartesianPointController());
man1.controller.enqueue_target([-2, 2, -pi/4], 8);
man2.controller.enqueue_target([2, 4, pi + pi/3], 8);

map1 = MapEstimator();
map2 = MapEstimator();

k_sens = 0;

for k = 1:length(t)
    man1.update_kinematic_dynamics();
    man2.update_kinematic_dynamics();
    
    if mod(k, 300) == 0 % Perform  a scan 
        k_sens = k_sens + 1
        scan1 = Scan(man1, cam1, obj);
        scan2 = Scan(man2, cam2, obj);
        figure(2), clf;
        plot(scan1);
        figure(3), clf;
        plot(scan2);
        
        map1.process_scan(man1, scan1, cam1);
        map2.process_scan(man2, scan2, cam2);

        [F1, a1] = map1.get_composite_informations();
        [F2, a2] = map2.get_composite_informations();

        for i_cons = 1:2
            % First linear consensus step
            map2.linear_consensus(F1, a1, 1/2, 1/2);
            % Second linear consensus step
            map1.linear_consensus(F2, a2, 2/3, 1/3);
        end

        map1.KF_update_step(2);
        map2.KF_update_step(2);

        newmap1 = map1.new_observations;
        newmap2 = map2.new_observations;

        if ~isempty(newmap1) || ~isempty(newmap2)
            newmap2.conditional_join(newmap1);
        end

        fprintf("Adding %d new states\n", newmap2.get_size());
        map1.join(newmap2);
        map2.join(newmap2);
        fprintf("Map1 size: %d\n", map1.get_size());
        fprintf("Map2 size: %d\n", map2.get_size());



        figure(4), clf, hold on;
        xlim([-4, 4]);
        ylim([-4, 4]);
        plot(map1);
        axis equal;
        figure(5), clf, hold on;
        axis equal;
        xlim([-4, 4]);
        ylim([-4, 4]);
        plot(map2);
 
   end

    if mod(k, 150) == 0
        figure(1), clf;
        plot(man1);
        plot(man2);
        plot(obj);
        xlim([-5, 12]);
        ylim([-3, 12]);
        axis equal
        title(sprintf('t = %.2f', t(k)));
    end
end


% figure(2), clf, hold on;
% plot(man2.error(1,:));
% plot(3*man2.sigma(1,:));
% plot(-3*man2.sigma(1,:));
% 
% figure(3), clf, hold on;
% plot(man2.error(3,:));
% plot(3*man2.sigma(3,:));
% plot(-3*man2.sigma(3,:));
% 
% figure(4), clf, hold on;
% plot(man2.d_error(3,:));
% plot(3*man2.d_sigma(3,:));
% plot(-3*man2.d_sigma(3,:));
% 
% figure(5), clf, hold on;
% plot(man2.d_error(1,:));
% plot(3*man2.d_sigma(1,:));
% plot(-3*man2.d_sigma(1,:));
