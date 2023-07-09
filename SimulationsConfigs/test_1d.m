start_simulation_loop;


test_name = "test_1d";
description = strcat("Problem summary:\n", ...
    "1 robot\n", ...
    "dynamic object\n", ...
    "collision avoidance strategy: correspondence based\n"...
    );

systems  = {
    System([-3; 1]); ...
    };

systems{1}.manipulator.set_initial_joint_config([pi/2; -pi/2 + randn()*0.2; -4/3*pi]);

obj_i = randi(numel(dataset));

with_dynamic_plot = true;
randomize_position = true;
guess_based_voronoi = true;


main_simulation_loop;
