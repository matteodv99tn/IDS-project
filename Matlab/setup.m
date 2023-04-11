
[conf_mkdir_status, msg] = mkdir("Configurations");

addpath("Classes");
addpath(fullfile("Classes", "Datasets"));
addpath(fullfile("Classes", "Controllers"));
addpath("Functions");
addpath("Configurations");



global dt;                      % sampling period for the simulation
global d_max;                   % maximum value that can be sensed 
global d_cam_std;               % standard deviation of the lidar scan ranging (in m)
global theta_cam_std;           % standard deviation of the lidar scan bearing (in rad)
global datasets;

dt              = 0.001;
d_max           = 30;
d_cam_std       = 0.01;
theta_cam_std   = 0.5 * pi / 180;   % 0.5 degrees

%% --- Default configuration

% --- Simulation parameters
config.simulation.dt    = 0.001;                            % sampling period for the simulation

% --- Camera parameters
config.camera.d_max     = 30;                               % maximum recordable distance value
config.camera.d_std     = 0.01;                             % distance estimation standard deviation
config.camera.theta_std = 0.1 * pi / 180;                   % angle estimation std

datasets = [Rectangle, Square, Pentagon, RegularPolygon(3), RegularPolygon(4), RegularPolygon(5), RegularPolygon(6),RegularPolygon(7),RegularPolygon(8),RegularPolygon(9)];
            
%% --- Export default configuration 
file = fopen(fullfile("Configurations", "default.json"), "w");
fprintf(file, "%s", jsonencode(config, PrettyPrint=true));
fclose(file);

global configuration_name;
configuration_name = "default";
fprintf("Remember to correctly override the desired configuration, currently:\n");
fprintf('    configuration_name = "%s";\n', configuration_name);
fprintf("The .json file extension is not required!\n");








