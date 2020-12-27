%% Multi vehicle with lidars
close all, clc, clear all
% - Init -
num_vehicles = 4;
sample_time = 0.1; % Time step [s]
sim_length = 20; % Simulation time [s]
addpath('map')

% Set initial speed for each vehicle:
init_vel = [26 30 35 40]./3.6;
init_ang = [-pi/16 -pi/50 -pi/16 -pi/16];
% init_ang = [0 0 0];
lane = [2 2 2 2];

% - Define vehicle -
wheel_radius = 0.05;  % Wheel radius [m]
f_length = 2;     % Distance from CG to front wheels [m]
r_length = 0.25;         % Distance from CG to rear wheels [m]
vehicle_model = FourWheelSteering(wheel_radius,[f_length r_length]);

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
    lidar.maxRange = 60;
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
    poses(:,v_idx) = [11.5+v_idx*20;246.5;init_ang(v_idx)];
end
% poses(1,3) = 10;
% poses(2,2) = 244
env.Poses = poses;
%% Simulation loop

% Create main data struct
for v_idx=1:num_vehicles
    vehicles(v_idx) = struct(...
        'pose',poses(:,v_idx),...
        'pose_prev',poses(:,v_idx),...
        'velocity',zeros(3,1),...
        'velocity_prev',zeros(3,1),...
        'ranges',zeros(length(lidar.scanAngles),1),...
        'ranges_prev',lidars{v_idx}(),... % Get lidar data
        'trailing_var',struct('kp',0.45,'kd',0.25,'t_hw_conn',2,'t_hw',0.1,'error',0),... % Variables for trailing alg
        'lane_keeping_var',struct('dist',f_length+2),... % Variables for lane keeping alg
        'parameters',struct('lane',lane(v_idx),'desired_vel',init_vel(v_idx),'conn',true,'sample_time',sample_time),...
        'messages',zeros(1,num_vehicles),... % Outgoing messages to other vehicles (each cell corresponds to a destination vehicle)
        'target',0);
end

allRanges = cell(1,num_vehicles);
for idx = 2:numel(time) % simulation loop
    
    % Get lidar range and execute controllers
    for v_idx = 1:num_vehicles
        ranges = lidars{v_idx}(); % Get lidar data
        vehicles(v_idx).ranges = ranges;
        allRanges{v_idx} = ranges;
        vehicles(v_idx) = swarmVehicleController(vehicles,v_idx);
    end
    
    % Update poses
    for v_idx = 1:num_vehicles
        vehicles(v_idx).pose = vehicles(v_idx).pose + vehicles(v_idx).velocity*sample_time; % Update
        poses(:,v_idx) = vehicles(v_idx).pose;
    end
    
    %Change lane
    if idx == 90
        vehicles(3).parameters.lane = 1;
        disp('Changed lane')
    end
    
    % Update visualizer
    env(1:num_vehicles,poses,allRanges)
    ylim([240 258])
    xlim([11 300])
end

%% Vehicle controller
function vehicle = swarmVehicleController(vehicles,v_id)
acc_on = false;
%TODO: Implement four wheel kinematic model (if we have time)
%TODO: Implement robot detectors (instead of lidar or with)
%TODO: Add swarm algorithm (~done)
%TODO:
% - add for y- axis


max_range = 1000; % [m]
vel_tresh = 5; % [km/h]
vehicle = vehicles(v_id);
num_vehicles = length(vehicles);

switch vehicle.parameters.conn
    case true % Connection
        
        %Init var
        min_diff_d_vel = 1000; 
        min_idx = -1;
        
        % Get target
        if vehicle.target == 0 % Check if vehicle already have a target
            for v_idx = 1:num_vehicles % 
                if v_idx == v_id
                    continue; % Skip ourselfs
                end
                diff_pose = vehicles(v_idx).pose(1) - vehicle.pose(1); % Difference in meters between vehicles (x-axis)
                %         diff_vel = vehicle.velocity(1:2) - vehicles(v_idx).velocity(1:2);
                diff_d_vel = vehicles(v_idx).parameters.desired_vel - vehicle.parameters.desired_vel;
                
                % Save vehicle with lowest relative velocity withinh a
                % certain range
                if (diff_d_vel < min_diff_d_vel) && (diff_pose < max_range) && (abs(diff_d_vel) <=vel_tresh/3.6) && (diff_pose >= 0)
                    min_diff_d_vel = diff_d_vel;
                    min_idx = v_idx;
                end
            end
            
        end
        
        %Update
        if  min_idx ~= -1
            vehicle.target = min_idx;
            disp(['Vehicle ' num2str(v_id) ' targets ' num2str(min_idx)])
        end
        
        % Follow target
        if vehicle.target ~= 0
            vehicle.parameters.lane = vehicles(vehicle.target).parameters.lane; % Change to target lane
            
            % CACC
            err_f = vehicles(vehicle.target).pose(1) - vehicle.pose(1) - vehicle.trailing_var.t_hw*vehicle.velocity(1); % e_k = x_k-1 - x_k - t_hw*v_k
            vx = vehicle.velocity_prev(1) + vehicle.trailing_var.kp*err_f + vehicle.trailing_var.kd*diff([vehicle.trailing_var.error err_f]); % vk_prev + k_p*e_k + k_d*e_k
            vehicle.trailing_var.error = err_f;
        else % Drive at defualt velocity 
            vx = vehicle.parameters.desired_vel;
        end
        
    case false % No connection
        % lidar scan angles: [0 pi/2 pi 3*pi/2]
        
        % Trailing with lidar
        if acc_on && vehicle.ranges(1) < 60
            % ACC
            a = 3*( vehicle.ranges(1) - vehicle.trailing_var.t_hw*vehicle.velocity(1)) + 1*(range_d); % Estimate required acceleration
            vx = a*vehicle.parameters.sample_time; % Calculate velocity
        else % Drive at defualt velocity 
            vx = vehicle.parameters.desired_vel;
        end
end
w = lane_keeper(vehicle); % Get turn angle from lane keeper

vehicle.velocity = bodyToWorld([vx;0;w], vehicle.pose); % To world coordinates
vehicle.ranges_prev = vehicle.ranges; % Save range
end


