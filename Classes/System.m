classdef System < handle

properties %% ---- Attributes of the class --------------------------------------------------------

    manipulator;
    scan;
    camera;
    map;
    planner;


    k;
    scans;

end % properties


methods %% ---- Member functions ------------------------------------------------------------------

    function self = System(RF)
        config = get_current_configuration();

        self.manipulator = Manipulator(3, 3, RF);
        self.camera = Camera();
        self.map = MapEstimator();
        self.planner = MotionPlanner();

        q0 = 2*pi*rand(3, 1) - pi;
        self.manipulator.set_initial_joint_config(q0);
        self.manipulator.set_controller(CartesianVelocityController());

        N = config.simulation.N_meas;
        self.k = 1;
        self.scans = cell(1, N);
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

        self.scans{self.k} = self.scan;
        self.k = self.k + 1;
    end

    function merge_data(self, newmap, F, a, N_robots)
        self.map.F = F;
        self.map.a = a;
        self.map.KF_update_step(N_robots);
        self.map.join(newmap);,
        self.planner.plan_motion(self.manipulator, self.map);
    end

    function state_idx = undesired_states(self)
        if ~isempty(self.planner.wrong_state_pos)
            for i = 1:self.map.get_size()
                if self.map.get_state_idx(i) == self.planner.wrong_state_pos
                    state_idx = i;
                    return;
                end
            end
        else
            state_idx = [];
        end
    end


    function constraint_voronoi_cell(self)

        function ps = circle(origin, r)
            theta = (0:19)*(2*pi/20);
            x = origin(1) + r*cos(theta);
            y = origin(2) + r*sin(theta);
            ps = polyshape(x, y);
        end

        max_bound = circle(self.manipulator.origin, ...
            (self.manipulator.L1 + self.manipulator.L2)*0.9);
        self.planner.allowed_region = intersect(self.planner.allowed_region, max_bound);

    end % constraint_voronoi_cell function



end % methods

end % System class
