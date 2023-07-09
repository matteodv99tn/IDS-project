start_simulation_loop;


test_name = "test_hexa_1c";
description = strcat("Problem summary:\n", ...
    "1 robot\n", ...
    "static object\n", ...
    "collision avoidance strategy: guess based\n"...
    );

systems  = {
    System([-3; 1]); ...
    };

systems{1}.manipulator.set_initial_joint_config([pi/2; -pi/2 + randn()*0.2; -4/3*pi]);

obj_i = 5;

with_dynamic_plot = true;
randomize_position = false;
guess_based_voronoi = true;


main_simulation_loop;
