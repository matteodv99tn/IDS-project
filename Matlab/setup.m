addpath("Classes");
addpath(fullfile("Classes", "Datasets"));
addpath("Functions");

global dt;                      % sampling period for the simulation
global d_max;                   % maximum value that can be sensed 
global d_cam_std;               % standard deviation of the lidar scan ranging (in m)
global theta_cam_std;           % standard deviation of the lidar scan bearing (in rad)

dt              = 0.01;
d_max           = 30;
d_cam_std       = 0.01
theta_cam_std   = 0.5 * pi / 180;   % 0.5 degrees
