%% Multi vehicle with lidars
close all, clc, clear all
% - Init -
num_vehicles = 3;
sample_time = 0.1; % Time step [s]
sim_length = 20; % Simulation time [s]
addpath('map')

% Set initial speed for each vehicle:
init_vel = [2 3 6];
init_ang = [-pi/16 -pi/50 -pi/16];
lane = [1 1 2];

% - Define vehicle -
wheel_radius = 0.05;  % Wheel radius [m]
f_length = 0.25;     % Distance from CG to front wheels [m]
r_length = 0.25;         % Distance from CG to rear wheels [m]
vehicle = FourWheelSteering(wheel_radius,[f_length r_length]);

% - Create environment -
env = MultiRobotEnv(num_vehicles);
env.robotRadius = f_length*4;
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
    lidar.maxRange = 20;
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
   poses(:,v_idx) = [11.5+v_idx*7;246.5;init_ang(v_idx)]; 
end
poses(2,3) = 255.5
poses(2,2) = 244
env.Poses = poses;
%% Simulation loop
allRanges = cell(1,num_vehicles);
ranges_prev = zeros(4,num_vehicles);
for v_idx = 1:num_vehicles
    ranges_prev(:,v_idx) = lidars{v_idx}();
end

vel = zeros(3,num_vehicles);
poses_prev = poses;
for idx = 2:numel(time) % simulation loop

    % Get lidar range and execute controllers
    for v_idx = 1:num_vehicles
        
        ranges = lidars{v_idx}(); % Get lidar data
        allRanges{v_idx} = ranges;
        vel(:,v_idx) = swarmVehicleController(poses(:,v_idx),poses_prev(:,v_idx),ranges,ranges_prev(:,v_idx),init_vel(v_idx),lane(v_idx));
        ranges_prev(:,v_idx) = ranges; % saverange
    end
    
    % Update poses
    poses_prev = poses;
    poses = poses + vel*sample_time;
    
    % Update visualizer
    env(1:num_vehicles,poses,allRanges)
    ylim([240 258])
    xlim([11 100])
end

%% Vehicle controller
function vel = swarmVehicleController(pose,pose_prev,ranges,ranges_prev,init_vel,lane)
    %TODO: implement four wheel kinematic model  
    dist =2; % meters to side [m]
    % lidar scan angles: [0 pi/2 pi 3*pi/2]
    a = 1;
    % Stay on the road
    switch lane
        case 1
            range = cos(pose(3))*ranges(4);
            range_prev = cos(pose_prev(3))*ranges_prev(4);
            range_d = diff([range range_prev]);
            
            err_right = range-dist;
            if range_d < 0.1
                a = 4;
            else
                a = 1;
            end
            w = -err_right*abs(range_d)*a;

        case 2
            range = cos(pose(3))*ranges(2);
            range_prev = cos(pose_prev(3))*ranges_prev(2);
            range_d = diff([range range_prev]);
            err_right = range-dist;
            w = err_right*abs(range_d)*1;
%             if ranges(2) < 1 % don't crash into left side
%                 w = -1;
%             elseif ranges(2) > 2
%                 w = 0.2;
%             else
%                 w =0;
%             end
    end

    if ranges(1) < 1
        vx = 1;
    else
        vx = init_vel;
    end
    vel = bodyToWorld([vx;0;w], pose);
end


