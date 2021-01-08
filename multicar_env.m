%% Multi vehicle simulation
close all, clc, clear all
% % - Init -
% num_vehicles = 4;
% sample_time = 0.1; % Time step [s]
% sim_length = 60; % Simulation time [s]
% addpath('map')
% 
% % Set initial speed for each vehicle:
% init_vel = [11 12 23 27]./3.6;
% init_ang = [-pi/16 -pi/50 -pi/16 -pi/16];
% % init_ang = [0 0 0];
% lane = [2 2 1 1];
load('test_case1')
max_acc = 5; % [m/s^2] max acceleration/deacceleration
% - Define vehicle -
wheel_radius = 0.05;  % Wheel radius [m]
f_length = 1;     % Distance from CG to front wheels [m]
r_length = 0.25;         % Distance from CG to rear wheels [m]
vehicle_model = FourWheelSteering(wheel_radius,[f_length r_length]);

% - Create environment -
env = MultiRobotEnv(num_vehicles);
env.robotRadius = f_length;
env.hasWaypoints = false;
env.showTrajectory = false;
env.plotSensorLines = true;
load map_v2;
close
env.mapName = 'map';

% - Create sensors for each vehicle -
lidars = cell(1,num_vehicles);
detectors = cell(1,num_vehicles);

for v_idx=1:num_vehicles
    
    % LIDAR
    lidar = MultiRobotLidarSensor;
    lidar.sensorOffset = [0,0];
    lidar.scanAngles = [0 pi/2 pi 3*pi/2];
    lidar.maxRange = 70;
    lidar.robotIdx = v_idx;
    lidars{v_idx} = lidar;
    attachLidarSensor(env,lidar);
    
    % RobotDetector
    detector = RobotDetector(env,v_idx);
    detector.sensorOffset = [0,0];
    detector.sensorAngle = 0;
    detector.fieldOfView = 2*pi;
    detector.maxRange = 50;
    detector.robotIdx = v_idx;
    detector.maxDetections = 4;
    detectors{v_idx} = detector;
    
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
    poses(:,v_idx) = [init_x_position(v_idx);init_y_position(v_idx);init_ang(v_idx)];
end
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
        'detections',detectors{v_idx}(),...
        'detections_prev',detectors{v_idx}(),...
        'trailing_var',struct('kp',0.45,'kd',0.25,'t_hw_conn',0.5,'t_hw',3,'error',0),... % Variables for trailing alg
        'lane_keeping_var',struct('dist',f_length+1),... % Variables for lane keeping alg
        'parameters',struct('lane',lane(v_idx),'desired_vel',init_vel(v_idx),'conn',init_conn(v_idx),'sample_time',sample_time,'max_range',100,'vel_tresh',5),...
        'messages',zeros(1,num_vehicles),... % Outgoing messages to other vehicles (each cell corresponds to a destination vehicle)
        'platoon_members',zeros(1,num_vehicles),...
        'isLeader', false,...
        'target',0);
end
allRanges = cell(1,num_vehicles);
for idx = 2:numel(time) % simulation loop
    % Get lidar range and execute controllers
    for v_idx = 1:num_vehicles
        % LiDAR
        ranges = lidars{v_idx}(); % Get lidar data
        vehicles(v_idx).ranges = ranges;
        allRanges{v_idx} = ranges;
        
        % Robotdetector
        detections = detectors{v_idx}();
        vehicles(v_idx).detections = detections;
        vehicles = swarmVehicleController(vehicles,v_idx,max_acc);
    end
    
    % Update poses
    for v_idx = 1:num_vehicles
        vehicles(v_idx).pose = vehicles(v_idx).pose + vehicles(v_idx).velocity*sample_time; % Update
        poses(:,v_idx) = vehicles(v_idx).pose;
    end
    
    % Check if approaching end of road
    for v_idx = 1:num_vehicles
        if vehicles(v_idx).pose(1) > 499
            for v_idx_2 = 1:num_vehicles % Check if any other vehicle targets this one
                if vehicles(v_idx_2).target == v_idx
                    vehicles(v_idx_2).target = 0; % Reset target
                    vehicles(v_idx_2).messages(v_idx) = 3; % notify target that it leaves platoon
                    disp(['Vehicle ' num2str(v_idx_2) ' lost ' num2str(v_idx)])
                end
            end
            vehicles(v_idx).pose(1) = 1; % Change pose
        end
    end
    if ~isempty(cl_ids) % Change lane
        for i=1:length(cl_ids)
            if idx == cl_times(i)
                vehicles(cl_ids(i)).parameters.lane = mod(vehicles(cl_ids(i)).parameters.lane,2)+1; % Change lane
                disp(['Vehicle ' num2str(cl_ids(i)) ' changed lane.'])
            end
        end
    end
    if ~isempty(bc_ids) % Break communcation
        for i=1:length(bc_ids)
            if idx == bc_times(i) && vehicles(bc_ids(i)).parameters.conn == 1
                vehicles(bc_ids(i)).parameters.conn = 0;
                disp(['Vehicle ' num2str(bc_ids(i)) ' lost communication.'])
            end
        end
    end
    if ~isempty(ec_ids) % Enable communication
        for i=1:length(ec_ids)
            if idx == ec_times(i) && vehicles(ec_ids(i)).parameters.conn == 0
                vehicles(ec_ids(i)).parameters.conn = 1;
                disp(['Vehicle ' num2str(ec_ids(i)) ' enabled communication.'])
            end
        end
    end
    % Update visualizer
    env(1:num_vehicles,poses,allRanges)
    ylim([20 30])
    xlim([0 500])
    set(gcf, 'Position',  [5, 500, 1900, 100]) % Set window position and size
    %     ylim([240 248])
    %     xlim([11 300])
end

%% Vehicle controller
function vehicles = swarmVehicleController(vehicles,v_id,max_acc)
acc_on = true;
%Done: Implement robot detectors (instead of lidar or with) 
%Done: Add swarm algorithm (~done)
%TODO:
% - add for y- axis
%TODO: Add automatic emergency braking
%TODO: Move code to functions

vehicle = vehicles(v_id);
num_vehicles = length(vehicles);

switch vehicle.parameters.conn
    case true % Connection
        
        %Init var
        min_diff_d_vel = 1000;
        min_idx = -1;
        
        % Handle messages
        for v_idx = 1:length(vehicles)
            if v_idx == v_id
                continue; % Skip ourselfs
            end
            % 1 = Incoming request to change lane
            % 2 = Request to change lane denied
            % 3 = Notified to join platoon
            % 4 = Notified to leave platoon
            % 5 = Notified by target that it has a other target
            % If in platoon the leader will update all the other vehicles
            % in the platoon of who is present.
            
            if vehicles(v_idx).messages(v_id) ~= 0 %Check for incoming message
                % Message received
                switch vehicles(v_idx).messages(v_id)
                    case 1 % Other vehicle asks us to change lane
                        if vehicle.target == 0 % Is not following
                            vehicle.parameters.lane = mod(vehicle.parameters.lane,2)+1; % Change lane
                            disp(['Vehicle ' num2str(v_id) ' changed lane to ' num2str(vehicle.parameters.lane)])
                            vehicle.parameters.lc_pose = vehicle.pose(1);
                        else
                            % Decline and answer the asking vehicle
                            disp(['Vehicle ' num2str(v_id) ' declines request'])
                            vehicle.messages(v_idx) = 2;
                        end
                    case 2 % Vehicle declines our request
                        disp(['Vehicle ' num2str(v_id) ' got declined, change lane'])
                        % Try to change lane
                        vehicle.parameters.lane = mod(vehicle.parameters.lane,2)+1; % Change lane
                    case 3 % Notified to join platoon
                            vehicle.platoon_members(v_idx) = true;
                            vehicle.isLeader = true;
                            vehicle.platoon_members(v_id) = true;
                    case 4 % Notified to leave platoon
                        vehicle.platoon_members(v_idx) = false;
                        if  ~all(vehicle.platoon_members) % No vehicles in platoon
                            vehicle.isLeader = false;
                        end
                    case 5 %Notified by target that it has another target
                        if vehicle.target == v_idx % Check that the message is from our target
                            vehicle.target = vehicles(v_idx).target; % Update our target
                            vehicle.messages(v_idx) = 4; % Notify old target to leave platoon
                            vehicle.messages(vehicle.target) = 3; % Notify new target to join platoon
                            disp(['Vehicle ' num2str(v_id) ' changes target to ' num2str(vehicle.target)])
                        end
                end
                vehicles(v_idx).messages(v_id) = 0; % reset value
            end
        end
        if vehicle.target ~= 0 % Do checks if conn is still established and if targets are correct
            % Check if conn established
            if vehicles(vehicle.target).parameters.conn == false
                vehicle.target = 0;
                vehicle.isLeader = false;
            else
                 % Check if someone targets me while i already have a target
                for v_idx = 1:num_vehicles % Get who targets me
                    if vehicles(v_idx).target == v_id
                        vehicle.messages(v_idx) = 5; % Notify that vehicle
                    end
                end
            end
        end
        
        % Get target
        if vehicle.target == 0 % Check if vehicle already have a target
            for v_idx = 1:num_vehicles %
                if v_idx == v_id || vehicles(v_idx).parameters.conn == false
                    continue; % Skip ourselfs or for those with no conn
                end
                diff_pose = vehicles(v_idx).pose(1) - vehicle.pose(1); % Difference in meters between vehicles (x-axis)
                %         diff_vel = vehicle.velocity(1:2) - vehicles(v_idx).velocity(1:2);
                diff_d_vel = vehicles(v_idx).parameters.desired_vel - vehicle.parameters.desired_vel;
                
                % Save vehicle with lowest relative velocity within a
                % certain range
                if (diff_d_vel < min_diff_d_vel) && (diff_pose < vehicle.parameters.max_range) && (abs(diff_d_vel) <=vehicle.parameters.vel_tresh/3.6) && (diff_pose >= 0)
                    min_diff_d_vel = diff_d_vel;
                    min_idx = v_idx;
                end
            end
        end
        %Update
        if  min_idx ~= -1
            vehicle.target = min_idx;
            vehicle.messages(vehicle.target) = 3; % notify target
            disp(['Vehicle ' num2str(v_id) ' targets ' num2str(min_idx)])
        end
        
        % Follow target
        if vehicle.target ~= 0
            vehicle.parameters.lane = vehicles(vehicle.target).parameters.lane; % Change to target lane
            
            % Check if there is other vehicles between us and the leader
            % if so follow that one
            rel_pose = 1000*ones(1,num_vehicles);
            for v_idx = 1:num_vehicles % Get relative position to all vehicles
                if vehicles(vehicle.target).platoon_members(v_idx) == true % Check if vehicle in platoon
                    rel_pose(v_idx) = vehicles(v_idx).pose(1) - vehicle.pose(1);
                    if rel_pose(v_idx) <= 0 % If vehicle ahead set that pose to a big positive number
                        rel_pose(v_idx) = 1000;
                    end
                end
            end
            [~, idx] = min(rel_pose);
            % CACC
            err_f = vehicles(idx).pose(1) - vehicle.pose(1) - vehicle.trailing_var.t_hw_conn*vehicle.velocity(1); % e_k = x_k-1 - x_k - t_hw*v_k
            vx = vehicle.velocity_prev(1) + vehicle.trailing_var.kp*err_f + vehicle.trailing_var.kd*diff([vehicle.trailing_var.error err_f]); % vk_prev + k_p*e_k + k_d*e_k
            vehicle.trailing_var.error = err_f;
        else % Drive at defualt velocity
            vx = vehicle.parameters.desired_vel;
        end
        
    case false % No connection
        if vehicle.target ~=0 % if it has target remove it
            vehicle.messages(vehicle.target) = 4; % notify target
            vehicle.target = 0;
        end
        if vehicle.isLeader
            
        end
        % lidar scan angles: [0 pi/2 pi 3*pi/2]

        % Trailing with lidar
        if acc_on && vehicle.ranges(1) < 70
            
            % ACC
            range_d = diff([vehicle.ranges_prev(1),vehicle.ranges(1)]);
            a = 3*( vehicle.ranges(1) - vehicle.trailing_var.t_hw*vehicle.velocity(1)) + 1*(range_d); % Estimate required acceleration
            vx = a*vehicle.parameters.sample_time; % Calculate velocity
        else % Drive at defualt velocity
            vx = vehicle.parameters.desired_vel;
        end
end
w = lane_keeper(vehicle); % Get turn angle from lane keeper
% Detect and react to close vehicles
if ~isempty(vehicle.detections) && ~isempty(vehicle.detections_prev)
    nmr_of_detections = size(vehicle.detections,1);
    for idx = 1:nmr_of_detections
        idx_prev = find(vehicle.detections_prev(:,3) == idx, 1);
        if  isempty(idx_prev)
            continue;
        end
        vehicle_exists = find(vehicle.detections(:,3) == idx, 1);
        if  ~isempty(vehicle_exists) && abs(vehicle.detections(idx_prev,2)) < pi/8 % Check if vehicle is in front and oncoming
            % ttc = d/(v_e - v_t)
            if vehicle.parameters.conn
                ttc = vehicle.detections(idx_prev,1)/(vehicle.velocity(1) - vehicles(idx).velocity(1)); %% ttc conn
            else
                ttc = vehicle.detections(idx_prev,1)*vehicle.parameters.sample_time/(vehicle.detections_prev(idx_prev,1)-vehicle.detections(idx_prev,1)); %% ttc with sensor
            end
            if vehicle.parameters.conn && ttc < 15 && ttc > 0 && isInSameLane(vehicle,vehicles(idx))
                if vehicle.target ~= 0
                    % Connection
                    if vehicles(idx).parameters.conn && vehicles(vehicle.target).platoon_members(idx) == false % Make sure it is not in the same platoon
                        disp(['Vehicle ' num2str(v_id) ' ask ' num2str(idx) ' to change lane'])
                        vehicle.messages(idx) = 1;
                    end
                elseif vehicles(idx).parameters.conn
                    % Connection but not in platoon
                    disp(['Vehicle ' num2str(v_id) ' ask ' num2str(idx) ' to change lane'])
                    vehicle.messages(idx) = 1;
                else
                    %Connectionless
                    disp(['Vehicle ' num2str(v_id) ' changes lane'])
                    % Try to change lane
                    vehicle.parameters.lane = mod(vehicle.parameters.lane,2)+1; % Change lane
                end
            end
            %                 if ttc < 4 && ttc > 0
            %                     disp('braking')
            %                     deacc = 8; % [m/s^2]
            %                     vx = vehicle.velocity(1)*0.5
            %
            %ACC
            %                     range_d = diff([vehicle.detections_prev(idx_prev,1),vehicle.detections(idx,1)]);
            %                     a = 3*( vehicle.detections(idx,1) - vehicle.trailing_var.t_hw*vehicle.velocity(1)) + 1*(range_d); % Estimate required acceleration
            %                     vx = a*vehicle.parameters.sample_time; % Calculate velocity
            %                 end
        end
    end
end

% Limit acceleration
acc = diff([vehicle.velocity(1), vx]);
if acc > max_acc
    vx = vehicle.velocity(1) + max_acc*vehicle.parameters.sample_time;
end
if acc < -max_acc
    vx = vehicle.velocity(1) - max_acc*vehicle.parameters.sample_time;
end
vehicle.velocity = bodyToWorld([vx;0;w], vehicle.pose); % To world coordinates
vehicle.ranges_prev = vehicle.ranges; % Save range
vehicle.detections_prev = vehicle.detections; % Save detections
vehicles(v_id) = vehicle; % Update current vehicle
end


