%% Multi vehicle with lidars
close all, clc, clear all
% - Init -
num_vehicles = 3;
sample_time = 0.1; % Time step [s]
sim_length = 20; % Simulation time [s]
addpath('map')

% Set initial speed for each vehicle:
init_vel = [5 5 3];
init_ang = [-pi/16 -pi/50 -pi/16];
init_ang = [0 0 0];
lane = [2 1 2];

% - Define vehicle -
wheel_radius = 0.05;  % Wheel radius [m]
f_length = 0.25;     % Distance from CG to front wheels [m]
r_length = 0.25;         % Distance from CG to rear wheels [m]
vehicle_model = FourWheelSteering(wheel_radius,[f_length r_length]);

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
    lidar.maxRange = 30;
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
   poses(:,v_idx) = [11.5+v_idx*9;246.5;init_ang(v_idx)]; 
end
% poses(2,3) = 255.5
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
       'trailing_var',struct('kp',0.45,'kd',0.25,'t_hw_conn',2,'t_hw',3,'error',0),... % Variables for trailing alg
       'lane_keeping_var',struct('dist',2),... % Variables for lane keeping alg
       'parameters',struct('lane',1,'desired_vel',init_vel(v_idx),'conn',false,'sample_time',sample_time),...
       'messages',zeros(1,num_vehicles)); % Outgoing messages to other vehicles (each cell corresponds to a destination vehicle)
end


allRanges = cell(1,num_vehicles);
% ranges_prev = zeros(4,num_vehicles);
% for v_idx = 1:num_vehicles
%     vehicles(v_idx).ranges_prev = lidars{v_idx}();
% %     ranges_prev(:,v_idx) = lidars{v_idx}();
% end
% vk = zeros(1,num_vehicles);
% err_f = zeros(1,num_vehicles);
% vel = zeros(3,num_vehicles);
% poses_prev = poses;
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
%     poses_prev = poses;
%     poses = poses + vel*sample_time;
    
    
    % Update visualizer
    env(1:num_vehicles,poses,allRanges)
    ylim([240 258])
    xlim([11 150])
end

%% Vehicle controller
function vehicle = swarmVehicleController(vehicles,v_idx)
    %TODO: implement four wheel kinematic model  
    vehicle = vehicles(v_idx);
    % lidar scan angles: [0 pi/2 pi 3*pi/2]
    a = 0.1;
    % Stay on the road
    switch vehicle.parameters.lane
        case 1
            range = cos(vehicle.pose(3))*vehicle.ranges(4);
            range_prev = cos(vehicle.pose_prev(3))*vehicle.ranges_prev(4);
            range_d = diff([range range_prev]);
            
            err_right = range-vehicle.lane_keeping_var.dist;
%             if range_d < 0.1
%                 a = 4;
%             else
%                 a = 1;
%             end
            w = -err_right*abs(range_d)*a;

        case 2
            range = cos(vehicle.pose(3))*vehicle.ranges(2);
            range_prev = cos(vehicle.pose_prev(3))*vehicle.ranges_prev(2);
            range_d = diff([range range_prev]);
            err_right = range-vehicle.lane_keeping_var.dist;
            w = err_right*abs(range_d)*1;
%             if ranges(2) < 1 % don't crash into left side
%                 w = -1;
%             elseif ranges(2) > 2
%                 w = 0.2;
%             else
%                 w =0;
%             end
    end
    
    % Trailing:
    % With Lidar
    vehicle.ranges(1)
    if vehicle.ranges(1) < 30
%         ttc = vehicle.ranges(1)/(range_d)
%         pose_x_target = vehicle.pose(1) + vehicle.ranges(1);
        % ACC
        a = 3*( vehicle.ranges(1) - vehicle.trailing_var.t_hw*vehicle.velocity(1)) + 1*(range_d); % Estimate required acceleration
        vk = a*vehicle.parameters.sample_time; % Calculate velocity
        
        % CACC
%         err_f = pose_x_target - vehicle.pose(1) - vehicle.trailing_var.t_hw*vehicle.velocity(1); % e_k = x_k-1 - x_k - t_hw*v_k 
%         vk = vehicle.velocity_prev(1) + vehicle.trailing_var.kp*err_f + vehicle.trailing_var.kd*diff([vehicle.trailing_var.error err_f]); % vk_prev + k_p*e_k + k_d*e_k
%         vehicle.trailing_var.error = err_f;
    else
        vk = vehicle.parameters.desired_vel;
    end
    
%     Brake
%     if vehicle.ranges(1) < 1
%         vk = vk*0.7;
%     end
        

    vehicle.velocity = bodyToWorld([vk;0;w], vehicle.pose);
    vehicle.ranges_prev = vehicle.ranges; % save range
end


