start_simulation_loop;


test_name = "test_3d";
description = strcat("Problem summary:\n", ...
    "3 robots\n", ...
    "dynamic object\n", ...
    "collision avoidance strategy: correspondence based\n"...
    );

systems  = {
    System([3; 3]); ...
    System([2; -4]); ...
    System([-3; 3]); ...
    };

systems{1}.manipulator.set_initial_joint_config([pi/6; -4/6*pi + randn()*0.2; 2*pi*rand()]);
systems{2}.manipulator.set_initial_joint_config([-5*pi/6; -5/6*pi + randn()*0.2; 2*pi*rand()]);
systems{3}.manipulator.set_initial_joint_config([5*pi/6; 4/6*pi + randn()*0.2; 2*pi*rand()]);

obj_i = randi(numel(dataset));

with_dynamic_plot = true;
randomize_position = true;
guess_based_voronoi = true;


main_simulation_loop;
