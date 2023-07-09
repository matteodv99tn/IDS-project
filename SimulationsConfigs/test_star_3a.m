start_simulation_loop;


test_name = "test_star_3a";
description = strcat("Problem summary:\n", ...
    "3 robots\n", ...
    "static object\n", ...
    "collision avoidance strategy: maximum size\n"...
    );

systems  = {
    System([3; 3]); ...
    System([2; -4]); ...
    System([-3; 3]); ...
    };

systems{1}.manipulator.set_initial_joint_config([pi/6; -4/6*pi + randn()*0.2; 2*pi*rand()]);
systems{2}.manipulator.set_initial_joint_config([-5*pi/6; -5/6*pi + randn()*0.2; 2*pi*rand()]);
systems{3}.manipulator.set_initial_joint_config([5*pi/6; 4/6*pi + randn()*0.2; 2*pi*rand()]);

obj_i = numel(dataset);

with_dynamic_plot = true;
randomize_position = false;
guess_based_voronoi = false;


main_simulation_loop;
