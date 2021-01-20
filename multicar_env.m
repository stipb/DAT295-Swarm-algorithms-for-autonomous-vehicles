%% Multi vehicle simulation
close all, clc, clear all

load('test_cases/test_case_many')

% - Define vehicle -
max_acc = 2.5; % [m/s^2] max acceleration/deacceleration
max_deacc = 5;
f_length = 1;     % Distance from CG to front wheels [m]
% wheel_radius = 0.05;  % Wheel radius [m]
% r_length = 0.25;         % Distance from CG to rear wheels [m]
% vehicle_model = FourWheelSteering(wheel_radius,[f_length r_length]);

% - Create environment -
env = MultiRobotEnv(num_vehicles);
env.robotRadius = f_length;
env.hasWaypoints = false;
env.showTrajectory = false;
env.plotSensorLines = false;
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
    lidar.scanAngles = [pi/2 3*pi/2];
    lidar.maxRange = 10;
    lidar.robotIdx = v_idx;
    lidars{v_idx} = lidar;
    attachLidarSensor(env,lidar);
    
    % RobotDetector
    detector = RobotDetector(env,v_idx);
    detector.sensorOffset = [0,0];
    detector.sensorAngle = 0;
    detector.fieldOfView = 2*pi;
    detector.maxRange = 60;
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
        'trailing_var',struct('kp',0.45,'kd',0.25,'t_hw_conn',-1.5,'t_hw',2,'error',0,'brake',false),... % Variables for trailing alg, time_gap = 2.5+t_hw
        'lane_keeping_var',struct('dist',f_length+0.65,'isChangingLane',false),... % Variables for lane keeping alg
        'parameters',struct('lane',lane(v_idx),'desired_vel',init_vel(v_idx),'conn',init_conn(v_idx),'sample_time',sample_time,'max_range',60,'vel_tresh',6),...
        'messages',zeros(1,num_vehicles),... % Outgoing messages to other vehicles (each cell corresponds to a destination vehicle)
        'platoon_members',zeros(1,num_vehicles),...
        'isLeader', false,...
        'target',0);
end
allRanges = cell(1,num_vehicles);

nmr_cols = ceil(num_vehicles/5);

for idx = 2:numel(time) % simulation loop
    tic
    if nmr_cols > 1
        nmr_veh = 5;
    else
        nmr_veh = num_vehicles;
    end
    if idx == 3
        % Init display
        v_ids = cell(1,num_vehicles);
        dots = cell(1,num_vehicles);
        conn_indicator = cell(1,num_vehicles);
        speed_indicator = cell(1,num_vehicles);
        target_indicator = cell(1,num_vehicles);
        text(10, 40, '.','FontSize',40,'Color','Magenta','Interpreter', 'latex')
        text(15, 36, 'Leader','FontSize',11,'Interpreter', 'latex')
        text(40, 40, '.','FontSize',40,'Color','Black','Interpreter', 'latex')
        text(45, 36, 'Not leader','FontSize',11,'Interpreter', 'latex')
        for col = 1:nmr_cols
            text(10+125*(col-1), 71, '\underline{Vehicle}','FontSize',12,'Interpreter', 'latex')
            text(35+125*(col-1), 71, '\underline{Connection}','FontSize',12,'Interpreter', 'latex') 
            text(69+125*(col-1), 71, '\underline{Velocity[km/h]}','FontSize',11,'Interpreter', 'latex') 
            text(106+125*(col-1), 71, '\underline{Target}','FontSize',11,'Interpreter', 'latex') 
            if col == nmr_cols
                nmr_veh = num_vehicles - (nmr_cols-1)*5;
            end
            for i=1:nmr_veh
                v_ids{i+(col-1)*5} = text(10+125*(col-1),71-i*5,['Vehicle ' num2str(i+(col-1)*5)],'Color','black');
                dots{i+(col-1)*5} = text(46+125*(col-1),74-i*5,'.','Fontsize',40,'Color','Green','Interpreter', 'latex');
                speed_indicator{i+(col-1)*5} = text(78+125*(col-1),71-i*5,'0','Fontsize',10);
                target_indicator{i+(col-1)*5} = text(111+125*(col-1),71-i*5,'-','Fontsize',10);
            end
        end
    end
    if idx > 3
        for col = 1:nmr_cols
            if col == nmr_cols
                nmr_veh = num_vehicles - (nmr_cols-1)*5;
            end
            for i=1:nmr_veh % Update display
                if vehicles(i+(col-1)*5).parameters.conn == true
                    set(dots{i+(col-1)*5}, 'Color','Green')
                else
                    set(dots{i+(col-1)*5}, 'Color','Red')
                end
                if vehicles(i+(col-1)*5).isLeader == true
                    set(v_ids{i+(col-1)*5}, 'Color','Magenta')
                else
                    set(v_ids{i+(col-1)*5}, 'Color','Black')
                end 
                
                set(speed_indicator{i+(col-1)*5},'String',num2str(vehicles(i+(col-1)*5).velocity(1)*3.6))
                if vehicles(i+(col-1)*5).target == 0
                    set(target_indicator{i+(col-1)*5},'String','-')
                else
                	set(target_indicator{i+(col-1)*5},'String',num2str(vehicles(i+(col-1)*5).target))
                end
            end
        end
    end
    % Get lidar range and execute controllers
    for v_idx = 1:num_vehicles
        % LiDAR
        ranges = lidars{v_idx}(); % Get lidar data
        vehicles(v_idx).ranges = ranges;
        allRanges{v_idx} = ranges;
        % Robotdetector
        detections = detectors{v_idx}(); % Get detector data
        vehicles(v_idx).detections = detections;
        vehicles = swarmVehicleController(vehicles,v_idx,max_acc,max_deacc);
    end
    % Update poses and color
    for v_idx = 1:num_vehicles
        vehicles(v_idx).pose = vehicles(v_idx).pose + vehicles(v_idx).velocity*sample_time; % Update
        poses(:,v_idx) = vehicles(v_idx).pose;
    end
    % Check if approaching end of road
    for v_idx = 1:num_vehicles
        if vehicles(v_idx).pose(1) > 490
            for v_idx_2 = 1:num_vehicles % Check if any other vehicle targets this one
                if vehicles(v_idx_2).target == v_idx
                    vehicles(v_idx_2).target = 0; % Reset target
                    vehicles(v_idx_2).messages(v_idx) = 4; % notify target that it leaves platoon
                    disp(['Vehicle ' num2str(v_idx_2) ' lost ' num2str(v_idx)])
                end
            end
            vehicles(v_idx).pose(1) = 1; % Change pose
        end
    end
    if ~isempty(cl_ids) % Change lane
        for i=1:length(cl_ids)
            if idx == cl_times(i)
                vehicles(cl_ids(i)) = change_lane(vehicles(cl_ids(i)), mod(vehicles(cl_ids(i)).parameters.lane,2)+1);
%                 vehicles(cl_ids(i)).parameters.lane = mod(vehicles(cl_ids(i)).parameters.lane,2)+1; % Change lane
%                 vehicles(cl_ids(i)).lane_keeping_var.isChangingLane = true;
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
    if ~isempty(cv_ids) % Change velocity
        for i=1:length(cv_ids)
            if idx == cv_times(i)
                vehicles(cv_ids(i)).parameters.desired_vel = cv_vel(i)/3.6;
                disp(['Vehicle ' num2str(cv_ids(i)) ' slows down to ' num2str(cv_vel(i)) ' [km/h]'])
            end
        end
    end
    
    % Update visualizer
    env(1:num_vehicles,poses,allRanges)
    ylim([20 30])
    xlim([0 500])
    set(gcf, 'Position',  [5, 500, 1900, 300]) % Set window position and size
    a = toc;
    if a < sample_time
        pause(sample_time-a)
    end
end

%% Vehicle controller
function vehicles = swarmVehicleController(vehicles,v_id,max_acc, max_deacc)
acc_on = true;
%Done: Implement robot detectors (instead of lidar or with) 
%Done: Add swarm algorithm (~done)
%TODO:
% - add for y- axis
%TODO: Add automatic emergency braking
%TODO: Move code to functions
vehicle = vehicles(v_id);
num_vehicles = length(vehicles);

% Check if lane change is complete
if vehicle.lane_keeping_var.isChangingLane
   if vehicle.parameters.lane == 2 && vehicle.pose(2) >= 28 - vehicle.lane_keeping_var.dist
       vehicle.lane_keeping_var.isChangingLane = false; % Manouver complete
       disp('-- Change lane to 2 complete --')
   end
   if v_id == 4
       as = 1;
   end
   if vehicle.parameters.lane == 1 && vehicle.pose(2) <= 21.6666 + vehicle.lane_keeping_var.dist
       vehicle.lane_keeping_var.isChangingLane = false; % Manouver complete
       disp('-- Change lane to 1 complete --')
   end
end


switch vehicle.parameters.conn
    case true % Connection
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
                            vehicle = change_lane(vehicle, mod(vehicle.parameters.lane,2)+1); % Try to change lane
                            disp(['Vehicle ' num2str(v_id) ' changed lane to ' num2str(vehicle.parameters.lane)])
                        else
                            % Decline and answer the asking vehicle
                            disp(['Vehicle ' num2str(v_id) ' declines request'])
                            vehicle.messages(v_idx) = 2;
                        end
                    case 2 % Vehicle declines our request
                        disp(['Vehicle ' num2str(v_id) ' got declined, change lane'])
                        % Try to change lane
                        vehicle = change_lane(vehicle, mod(vehicle.parameters.lane,2)+1);
                    case 3 % Notified to join platoon
                            vehicle.platoon_members(v_idx) = true; % Add that vehicle
                            vehicle.isLeader = true;
                            vehicle.platoon_members(v_id) = true; % Add ourelfs
                    case 4 % Notified to leave platoon
                        vehicle.platoon_members(v_idx) = false;
                        platoon_members_temp = vehicle.platoon_members;
                        platoon_members_temp(v_id) = 0;
                        if  ~any(platoon_members_temp) % No vehicles in platoon
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
            vehicle.isLeader = false;
            % Check if conn established
            if vehicles(vehicle.target).parameters.conn == false
                vehicle.target = 0;
            else
                 % Check if someone targets me while i already have a target
                for v_idx = 1:num_vehicles % Get who targets me
                    if vehicles(v_idx).target == v_id
                        vehicle.messages(v_idx) = 5; % Notify that vehicle
                    end
                end
            end
        end
        %Init var for choosing targets
        min_diff_d_vel = 1000;
        min_idx = -1;
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
            if vehicle.parameters.lane ~= vehicles(vehicle.target).parameters.lane &&...
                    vehicles(vehicle.target).pose(1)-vehicle.pose(1) < 50
                vehicle = change_lane(vehicle,vehicles(vehicle.target).parameters.lane); % Try to change lane
            end
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
        else % drive with acc
            vx = vehicle.parameters.desired_vel; % Set defualt speed
            % Trailing with robot detector
            if ~isempty(vehicle.detections) && ~isempty(vehicle.detections_prev)
    %             nmr_of_detections = size(vehicle.detections,1);
                for idx = 1:num_vehicles
                    idx_curr = find(vehicle.detections(:,3) == idx, 1);
                    idx_prev = find(vehicle.detections_prev(:,3) == idx, 1);
                    if  isempty(idx_curr) || isempty(idx_prev)
                        continue;
                    end

                    if vehicle.detections(idx_curr,1) < 60 && abs(vehicle.detections(idx_curr,2)) < pi/32 % Check if vehicle is in front
                        % ACC
                        velocity_proceeding_veh = (vehicle.detections_prev(idx_prev,1)- vehicle.detections(idx_curr,1))/vehicle.parameters.sample_time;
                        a = 0.23*(vehicle.detections(idx_curr,1)-vehicle.trailing_var.t_hw*vehicle.velocity(1)) + ...
                            0.07*(velocity_proceeding_veh);
                        vx = vehicle.velocity(1) + a*vehicle.parameters.sample_time; % Calculate velocity
                        if vx < vehicle.parameters.desired_vel
                            vx = vx + 0.5*(vehicle.parameters.desired_vel-vx);
                        end
                    end
                end
            end
        end
    case false % No connection
        if vehicle.target ~=0 % if it has target remove it
            vehicle.messages(vehicle.target) = 4; % notify target
            vehicle.target = 0;
        end
        if vehicle.isLeader
            for v_idx = 1:num_vehicles % Check if any other vehicle targets this one
                if vehicles(v_idx).target == v_id
                    vehicles(v_idx).target = 0; % Reset target
                end
            end
            vehicle.platoon_members(:) = false; % Clear platoon matrix
        end
        vx = vehicle.parameters.desired_vel; % Set defualt speed
        % Trailing with robot detector
        if ~isempty(vehicle.detections) && ~isempty(vehicle.detections_prev)
%             nmr_of_detections = size(vehicle.detections,1);
            for idx = 1:num_vehicles
                idx_curr = find(vehicle.detections(:,3) == idx, 1);
                idx_prev = find(vehicle.detections_prev(:,3) == idx, 1);
                if  isempty(idx_curr) || isempty(idx_prev)
                    continue;
                end
                if vehicle.detections(idx_curr,1) < 60 && abs(vehicle.detections(idx_curr,2)) < pi/32 % Check if vehicle is in front
                    % ACC
                    velocity_proceeding_veh = (vehicle.detections_prev(idx_prev,1)- vehicle.detections(idx_curr,1))/vehicle.parameters.sample_time;
                    a = 0.23*(vehicle.detections(idx_curr,1)-vehicle.trailing_var.t_hw*vehicle.velocity(1)) + ...
                        0.07*(velocity_proceeding_veh);
                    vx = vehicle.velocity(1) + a*vehicle.parameters.sample_time; % Calculate velocity

                end
            end
        end
end
w = lane_keeper(vehicle); % Get turn angle from lane keeper
% Detect and react to close vehicles
if ~isempty(vehicle.detections) && ~isempty(vehicle.detections_prev)
%     nmr_of_detections = size(vehicle.detections,1);
    for idx = 1:num_vehicles
        idx_curr = find(vehicle.detections(:,3) == idx, 1);
        idx_prev = find(vehicle.detections_prev(:,3) == idx, 1);
        if  isempty(idx_curr) || isempty(idx_prev)
            continue;
        end
        % ttc = d/(v_e - v_t)
        if vehicle.parameters.conn && vehicles(idx).parameters.conn
            ttc = vehicle.detections(idx_curr,1)/(vehicle.velocity(1) - vehicles(idx).velocity(1)); %% ttc conn
        else
            ttc = vehicle.detections(idx_curr,1)*vehicle.parameters.sample_time/(vehicle.detections_prev(idx_prev,1)-vehicle.detections(idx_curr,1)); %% ttc with sensor
        end
%         if v_id ==1 && isInSameLane(vehicle,vehicles(idx))
%             a = 1;
%         end
        if ttc < 3.5 && ttc > 0 && vehicle.lane_keeping_var.isChangingLane &&...
                isInSameLane(vehicle,vehicles(idx)) %&& abs(vehicle.detections(idx_curr,2))>pi/2
           % Cancel lane change manouver if ttc low and vehicle is behind
           % us
           vehicle.parameters.lane = mod(vehicle.parameters.lane,2)+1;
           vehicle.trailing_var.brake = true;
           disp('Canceled manouver due to low ttc')
        end
        if vehicle.trailing_var.brake
            vx = max_acc*vehicle.parameters.sample_time;
            if vehicle.velocity(1) <= 40/3.6 % CHANGE THIS TO BRAKE TO LOWEST SPEED OF VEHICLE INFRONT
                vehicle.trailing_var.brake = false;
            end
        end
        
        if abs(vehicle.detections(idx_curr,2)) < pi/16 % Check if vehicle is in front
            if ttc < 3 && ttc > 0 && isInSameLane(vehicle,vehicles(idx))
                    velocity_proceeding_veh = (vehicle.detections_prev(idx_prev,1)- vehicle.detections(idx_curr,1))/vehicle.parameters.sample_time;
                    a = 0.23*(vehicle.detections(idx_curr,1)-vehicle.trailing_var.t_hw*vehicle.velocity(1)) + ...
                        0.07*(velocity_proceeding_veh);
                    vx = vehicle.velocity(1) + a*vehicle.parameters.sample_time; % Calculate velocity
                    if ttc > 5.5 && isInSameLane(vehicle,vehicles(idx)) && vehicles(vehicle.target).platoon_members(idx) == false
                        vehicle = change_lane(vehicle, mod(vehicle.parameters.lane,2)+1); % Try to change lane
                        disp(['Vehicle ' num2str(v_id) ' changes lane to ' num2str(vehicle.parameters.lane) ' Vehicle ' num2str(idx) ' is too close' ])
                    end
                    
            elseif ttc < 4 && ttc > 0 && isInSameLane(vehicle,vehicles(idx))
                if vehicle.parameters.conn
                    % Connection
                    if vehicles(idx).parameters.conn && vehicle.target ~= 0 && vehicles(vehicle.target).platoon_members(idx) == false % Make sure it is not in the same platoon
                        disp(['Vehicle ' num2str(v_id) ' ask ' num2str(idx) ' to change lane'])
                        vehicle.messages(idx) = 1;
                    elseif vehicles(idx).parameters.conn && vehicle.isLeader == true % If leader ask vehicle to change lane 
                        disp(['Vehicle ' num2str(v_id) ' ask ' num2str(idx) ' to change lane'])
                        vehicle.messages(idx) = 1;
                    elseif vehicle.target ~= idx
                    disp(['Vehicle ' num2str(v_id) ' changes lane'])
                    % Try to change lane
                    vehicle = change_lane(vehicle, mod(vehicle.parameters.lane,2)+1);
                    end
                else
                    %Connectionless
                    disp(['Vehicle ' num2str(v_id) ' changes lane'])
                    % Try to change lane
                    vehicle = change_lane(vehicle, mod(vehicle.parameters.lane,2)+1);
                end
            end
        end
    end
end
% Limit acceleration
acc = diff([vehicle.velocity(1), vx]);
if acc > max_acc
    vx = vehicle.velocity(1) + max_acc*vehicle.parameters.sample_time;
end
if acc < -max_deacc
    vx = vehicle.velocity(1) - max_deacc*vehicle.parameters.sample_time;
end
vehicle.velocity = bodyToWorld([vx;0;w], vehicle.pose); % To world coordinates
vehicle.ranges_prev = vehicle.ranges; % Save range
vehicle.detections_prev = vehicle.detections; % Save detections
vehicles(v_id) = vehicle; % Update current vehicle
end