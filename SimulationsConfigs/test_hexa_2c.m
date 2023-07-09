start_simulation_loop;


test_name = "test_hexa_2c";
description = strcat("Problem summary:\n", ...
    "2 robots\n", ...
    "static object\n", ...
    "collision avoidance strategy: guess_based\n"...
    );

systems  = {
    System([3; 3]); ...
    System([2; -4]); ...
    };

systems{1}.manipulator.set_initial_joint_config([pi/6; -4/6*pi + randn()*0.2; 2*pi*rand()]);
systems{2}.manipulator.set_initial_joint_config([-5*pi/6; -5/6*pi + randn()*0.2; 2*pi*rand()]);

obj_i = 5;

with_dynamic_plot = true;
randomize_position = false;
guess_based_voronoi = true;


main_simulation_loop;
