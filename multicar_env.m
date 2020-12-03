%% Multi vehicle with lidars
close all, clc, clear all
% - Init -
num_vehicles = 3;
sample_time = 0.1; % Time step [s]
sim_length = 20; % Simulation time [s]
addpath('map')
% Set initial speed for each vehicle:
init_vel = [3 2 6];
init_ang = [-pi/12 0 pi/12];
% - Define vehicle -
wheel_radius = 0.05;  % Wheel radius [m]
f_length = 0.25;     % Distance from CG to front wheels [m]
r_length = 0.25;         % Distance from CG to rear wheels [m]
vehicle = FourWheelSteering(wheel_radius,[f_length r_length]);

% - Create environment -
env = MultiRobotEnv(num_vehicles);
env.robotRadius = f_length;
env.hasWaypoints = false;
env.plotSensorLines = true;
load map;
close
env.mapName = 'map';

% - Create sensors for each vehicle - 
lidars = cell(1,num_vehicles);

for v_idx=1:num_vehicles
    lidar = MultiRobotLidarSensor;
    lidar.sensorOffset = [0,0];
    lidar.scanAngles = [0 pi/2 pi 3*pi/2];
    lidar.maxRange = 10;
    lidar.robotIdx = v_idx;
    lidars{v_idx} = lidar;
    attachLidarSensor(env,lidar);
end




%% Prep simulation
time = 0:sample_time:sim_length; % Time array
poses = zeros(3,num_vehicles);
% Pose structure:
% [x_1, x_2, ...,x_num_vehicles]
% [y_1, y_2, ...,y_num_vehicles]
% [a_1, a_2, ...,a_num_vehicles]

% - Define initial pose -
for v_idx = 1:num_vehicles
   poses(:,v_idx) = [11.5+v_idx*3;246.5;init_ang(v_idx)]; 
end
env.Poses = poses;
%% Simulation loop
allRanges = cell(1,num_vehicles);
for idx = 2:numel(time)

    % Get lidar range and execute controllers
    for v_idx = 1:num_vehicles
        ranges = lidars{v_idx}(); % Get lidar data
        allRanges{v_idx} = ranges;
        vel(:,v_idx) = swarmVehicleController(poses(:,v_idx),ranges,init_vel(v_idx));
        
    end
    
    % Update poses
    poses = poses + vel*sample_time;
    
    % Update visualizer
    env(1:num_vehicles,poses,allRanges)
    ylim([240 258])
    xlim([11 100])
end

%% Vehicle controller
function vel = swarmVehicleController(pose,ranges,init_vel)
    lane = 1;
    %TODO: implement four wheel kinematic model

    % lidar scan angles: [0 pi/2 pi 3*pi/2]

    % Stay on the road
    switch lane
        case 1
            if ranges(4) < 1 % don't crash into right side
                w = 1;
            elseif ranges(4) > 2
                w = -0.2;
            else
                w = 0; 
            end
        case 2
            if ranges(2) < 1 % don't crash into right side
                w = -1;
            elseif ranges(2) > 2
                w = 0.2;
            else
                w =0;
            end
    end

    if ranges(1) < 1
        vx = 1;
    else
        vx = init_vel;
    end
    vel = bodyToWorld([vx;0;w], pose);
end


