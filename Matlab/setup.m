
[conf_mkdir_status, msg] = mkdir("Configurations");

addpath("Classes");
addpath(fullfile("Classes", "Datasets"));
addpath(fullfile("Classes", "Controllers"));
addpath("Functions");
addpath("Configurations");
addpath("Test_Scripts");


global dt;                      % sampling period for the simulation
global d_max;                   % maximum value that can be sensed 
global d_cam_std;               % standard deviation of the lidar scan ranging (in m)
global theta_cam_std;           % standard deviation of the lidar scan bearing (in rad)

dt              = 0.001;
d_max           = 30;
d_cam_std       = 0.01;
theta_cam_std   = 0.5 * pi / 180;   % 0.5 degrees


%% --- Dataset of feasible objects
dataset = {
    Rectangle(), ...
    RegularPolygon(3), ... 
    RegularPolygon(4), ...
    RegularPolygon(5), ...
    RegularPolygon(6), ...
    RegularPolygon(7), ...
    RegularPolygon(8)  ... 
    };


%% --- Default configuration

% --- Simulation parameters
config.simulation.dt    = 0.001;                        % sampling period for the simulation

% --- Feature extraction algorithm
config.feature_extraction.delta = 0.03;
config.feature_extraction.epsilon = 0.02;
config.feature_extraction.max_alpha = 10 * pi / 180;
config.feature_extraction.n_point_generation = 6; 
config.feature_extraction.n_point_back_on_success = 2; 
config.feature_extraction.min_seed_length = 0.05;
config.feature_extraction.min_seed_n_points = 8;

% --- Camera parameters
config.camera.d_max             = 30;                   % maximum recordable distance value
config.camera.d_std             = 0.005;                % distance estimation standard deviation
config.camera.theta_std         = 0.05 * pi / 180;      % angle estimation std;
config.camera.defaults.fov      = 60 * pi / 180;        % default field of view
config.camera.defaults.n_points = 151;                  % default amount of points per scan


% --- Manipulator parameters 
config.manipulator.std_position = 0.005 * pi / 180;      % std of the position measurement
config.manipulator.std_velocity = 0.0001 * pi / 180;       % std of the velocity measurement


% --- Estimator parameters
config.estimator.buffer_size = 4;
config.estimator.mahalanobis_th = 3;

% --- Motion planner
config.planner.search_th = 0.1;
config.planner.r_target = 0.1;
config.planner.v_des = 0.4;
config.planner.kp_radial = 0.5;

            
%% --- Export default configuration 
file = fopen(fullfile("Configurations", "default.json"), "w");
fprintf(file, "%s", jsonencode(config, PrettyPrint=true));
fclose(file);

global configuration_name;
configuration_name = "default";
fprintf("Remember to correctly override the desired configuration, currently:\n");
fprintf('    configuration_name = "%s";\n', configuration_name);
fprintf("The .json file extension is not required!\n");








