classdef System < handle

properties %% ---- Attributes of the class --------------------------------------------------------

    manipulator;
    scan;
    camera;
    map;
    planner;

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = System(RF)
        self.manipulator = Manipulator(3, 3, RF);
        self.camera = Camera();
        self.map = MapEstimator();
        self.planner = MotionPlanner();

        self.manipulator.set_initial_joint_config([0; 1; -1]);
        self.manipulator.set_controller(CartesianVelocityController());
    end % System constructor


    function plot(self, idx)
        figure(idx), clf, hold on;
        plot(self.manipulator);
        axis equal;
        xlim([-10, 10]);
        ylim([-10, 10]);
        grid on;

        figure(idx+100), clf, hold on;
        plot(self.scan);
        axis equal;
        xlim([-1, 10]);
        ylim([-5, 5]);
        grid on;

        figure(idx+200), clf, hold on;
        plot(self.map);
        axis equal;
        xlim([-5, 5]);
        ylim([-5, 5]);
        grid on;
    end % plot function


    function update(self)
        self.planner.update_target(self.manipulator, self.map);
        self.manipulator.update_kinematic_dynamics();
    end


    function [newmap, F, a] = scan_object(self, obj)
        self.scan = Scan(self.manipulator, self.camera, obj);
        self.map.process_scan(self.manipulator, self.scan, self.camera);
        [F, a] = self.map.get_composite_informations();
        newmap = self.map.new_observations;
    end

    function merge_data(self, newmap, F, a, N_robots)
        self.map.F = F;
        self.map.a = a;
        self.map.KF_update_step(N_robots);
        self.map.join(newmap);,
        self.planner.plan_motion(self.manipulator, self.map);
    end



end % methods

end % System class
