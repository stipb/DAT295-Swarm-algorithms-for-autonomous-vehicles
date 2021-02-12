%% Multi vehicle simulation
close all

% COMMENT THIS IF RUNNING AUTOMATIC TESTS!
clear all

load('test_cases/test_case_9') 
% Flags
save_as_video = true;
save_data = true; % Flag to save data
timed = true; % Try to match loop time with actual time
% - Define vehicle -
max_acc = 2.5; % [m/s^2] max acceleration
max_deacc = 5; % [m/s^2] max deacceleration
f_length = 1;     % Distance from CG to front wheels [m]
% - init video recorder - 
if save_as_video
    vidObj = VideoWriter(['video_files/' test_name],'MPEG-4');
    open(vidObj)
end
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
    lidar.sensorOffset = [1,0];
    lidar.scanAngles = [pi/2 3*pi/2];
    lidar.maxRange = 7;
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
    detector.maxDetections = 10;
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
%% Init data variables
if save_data == true
    velocities = zeros(length(time),num_vehicles,3);  %3D-matrix
    veh_counter = 0;
    timeToTarget = [];
end
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
        'trailing_var',struct('kp',0.45,'kd',0.25,'t_hw_conn',-1.5,'t_hw',2.5,'error',0,'brake',false),... % Variables for trailing alg, time_gap = 2.5+t_hw_conn
        'lane_keeping_var',struct('dist',f_length+0.65,'isChangingLane',false),... % Variables for lane keeping alg
        'parameters',struct('lane',lane(v_idx),'desired_vel',init_vel(v_idx),'conn',init_conn(v_idx),'sample_time',sample_time,'max_range',60,'vel_tresh',15),...
        'messages',zeros(1,num_vehicles),... % Outgoing messages to other vehicles (each cell corresponds to a destination vehicle)
        'lastDecline',0,...
        'lastChangeTry',0,...
        'platoon_members',zeros(1,num_vehicles),...
        'isLeader', false,...
        'target',0,...
        'timings',struct('timeGotTarget',0,'timeToTarget',0));
end
allRanges = cell(1,num_vehicles);
nmr_cols = ceil(num_vehicles/5);
for t_idx = 2:numel(time) % simulation loop
    tic % Start timer for simulation loop
    if nmr_cols > 1
        nmr_veh = 5;
    else
        nmr_veh = num_vehicles;
    end
    if t_idx == 3
        % Init display
        clock = text(10,8,'Time 0.00'); text(34,8,'[s]')
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
    if t_idx > 3
        set(clock,'String',['Time: ' num2str(time(t_idx))])
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
                
                set(speed_indicator{i+(col-1)*5},'String',num2str(vehicles(i+(col-1)*5).velocity(1)*3.6,2))
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
        vehicles = swarmVehicleController(vehicles,v_idx,max_acc,max_deacc,time(t_idx),init_vel);
    end
    % Update poses and color and save data
    for v_idx = 1:num_vehicles
        vehicles(v_idx).pose = vehicles(v_idx).pose + vehicles(v_idx).velocity*sample_time; % Update
        poses(:,v_idx) = vehicles(v_idx).pose;
        if save_data == true % Save data
            velocities(t_idx,v_idx,:) = vehicles(v_idx).velocity;
            if vehicles(v_idx).timings.timeToTarget ~= 0
                timeToTarget = [timeToTarget; vehicles(v_idx).timings.timeToTarget]; % Save data
                vehicles(v_idx).timings.timeToTarget = 0;
                vehicles(v_idx).timings.timeGotTarget = 0;
            end
        end
    end
    % Check if approaching end of road
    for v_idx = 1:num_vehicles
        if vehicles(v_idx).pose(1) > 490
            for v_idx_2 = 1:num_vehicles % Check if any other vehicle targets this one
                if vehicles(v_idx_2).target == v_idx
                    vehicles(v_idx_2).target = 0; % Reset target
                    vehicles(v_idx_2).messages(v_idx) = 4; % notify target that it leaves platoon
%                     disp(['Vehicle ' num2str(v_idx_2) ' lost ' num2str(v_idx)])
                end
            end
            if save_data == true
               veh_counter = veh_counter + 1; % Extract how many vehicles starts over for throughput
            end
            vehicles(v_idx).pose(1) = 1; % Change pose
        end
    end
    if ~isempty(cl_ids) % Change lane
        for i=1:length(cl_ids)
            if time(t_idx) == cl_times(i)
                vehicles(cl_ids(i)) = change_lane(vehicles(cl_ids(i)), mod(vehicles(cl_ids(i)).parameters.lane,2)+1);
                vehicles(cl_ids(i)).parameters.lane = mod(vehicles(cl_ids(i)).parameters.lane,2)+1; % Change lane
                vehicles(cl_ids(i)).lane_keeping_var.isChangingLane = true;
                disp(['Vehicle ' num2str(cl_ids(i)) ' changed lane.'])
            end
        end
    end
    if ~isempty(bc_ids) % Break communcation
        for i=1:length(bc_ids)
            if time(t_idx) == bc_times(i) && vehicles(bc_ids(i)).parameters.conn == 1
                vehicles(bc_ids(i)).parameters.conn = 0;
                disp(['Vehicle ' num2str(bc_ids(i)) ' lost communication.'])
            end
        end
    end
    if ~isempty(ec_ids) % Enable communication
        for i=1:length(ec_ids)
            if time(t_idx) == ec_times(i) && vehicles(ec_ids(i)).parameters.conn == 0
                vehicles(ec_ids(i)).parameters.conn = 1;
                disp(['Vehicle ' num2str(ec_ids(i)) ' enabled communication.'])
            end
        end
    end
    if ~isempty(cv_ids) % Change velocity
        for i=1:length(cv_ids)
            if time(t_idx) == cv_times(i)
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
    if timed && a < sample_time
        pause(sample_time-a)
    end
    if save_as_video
    	% Get it as an avi-frame
    	F = getframe(gcf);
        % Add the frame to the avi
        writeVideo(vidObj,F);
    end
end
close(vidObj)
%% Plot data
if save_data == true
    disp('-- Results --')
    plot_data(velocities,num_vehicles,time)
    throughput = veh_counter/sim_length;
    disp(['Throughput: ' num2str(throughput*60) '[ vehicles/min]'])
    fuelConsumption
    disp(['Mean time to target: ' num2str(mean(timeToTarget)) ' [s]'])
    disp('-------------')
end
%% Vehicle controller
function vehicles = swarmVehicleController(vehicles,v_id,max_acc, max_deacc,time,init_vel)
time_out = 3; % ask for lane change timeout [s]
vehicle = vehicles(v_id); % Get current vehicle
num_vehicles = length(vehicles);

% Check if lane change is complete
if vehicle.lane_keeping_var.isChangingLane
   if vehicle.parameters.lane == 2 && vehicle.pose(2) >= 27 - vehicle.lane_keeping_var.dist
       vehicle.lane_keeping_var.isChangingLane = false; % Manouver complete
%        disp('-- Change lane to 2 complete --')
   end
   if vehicle.parameters.lane == 1 && vehicle.pose(2) <= 22.6666 + vehicle.lane_keeping_var.dist
       vehicle.lane_keeping_var.isChangingLane = false; % Manouver complete
%        disp('-- Change lane to 1 complete --')
   end
end
    
switch vehicle.parameters.conn
    case true % Connection
        % Handle messages
        for v_idx = 1:length(vehicles)
            if v_idx == v_id
                continue; % Skip ourselfs
            end
            if vehicles(v_idx).messages(v_id) ~= 0 %Check for incoming message
                % Message received
                % 1 = Incoming request to change lane
                % 2 = Request to change lane denied
                % 3 = Notified to join platoon
                % 4 = Notified to leave platoon
                % 5 = Notified by target that it has a other target
                % If in platoon the leader will update all the other vehicles
                % in the platoon of who is present.
                switch vehicles(v_idx).messages(v_id)
                    case 1 % Other vehicle asks us to change lane
                        if vehicle.target == 0 && ~vehicle.isLeader && ~vehicle.lane_keeping_var.isChangingLane% Is not following and not currently changing lane
                            vehicle = change_lane(vehicle, mod(vehicle.parameters.lane,2)+1); % Try to change lane
%                             disp(['Vehicle ' num2str(v_id) ' changes lane to ' num2str(vehicle.parameters.lane)])
                        elseif vehicle.isLeader && vehicle.parameters.desired_vel <  vehicles(v_idx).parameters.desired_vel% Compare desired velocity (lowest changes lane)
                            vehicle = change_lane(vehicle, mod(vehicle.parameters.lane,2)+1); % Try to change lane
                        else
                            % Decline and answer the asking vehicle
%                             disp(['Vehicle ' num2str(v_id) ' declines request'])
                            vehicle.messages(v_idx) = 2;
                        end
                    case 2 % Vehicle declines our request
%                         disp(['Vehicle ' num2str(v_id) ' got declined, change lane'])
                        vehicle.lastDecline = time; % save timepoint for timeout
                        vehicle = change_lane(vehicle, mod(vehicle.parameters.lane,2)+1);% Try to change lane
                    case 3 % Notified to join platoon
                            vehicle.platoon_members(v_idx) = true; % Add that vehicle
                            vehicle.isLeader = true; % Set ourselfs to leader
                            vehicle.platoon_members(v_id) = true; % Add ourelfs
                    case 4 % Notified to leave platoon
                        vehicle.platoon_members(v_idx) = false; % Remove that member
                        
                        % Check if there is still other members in platoon
                        platoon_members_temp = vehicle.platoon_members;
                        platoon_members_temp(v_id) = 0;
                        if  ~any(platoon_members_temp) % No vehicles in platoon
                            vehicle.isLeader = false; % Remove ourselfs as leader
                        end
                    case 5 %Notified by target that it has another target
                        if vehicle.target == v_idx && vehicles(v_idx).target ~= 0% Check that the message is from our target and verify
                            % it has another target
                            diff_d_vel = vehicles(vehicles(v_idx).target).parameters.desired_vel - vehicle.parameters.desired_vel;
                            if abs(diff_d_vel) <=vehicle.parameters.vel_tresh/3.6  % check if within desired velocity
                                vehicle.target = vehicles(v_idx).target; % Update our target
                                vehicle.messages(v_idx) = 4; % Notify old target to leave platoon
                                vehicle.messages(vehicle.target) = 3; % Notify new target to join platoon
%                                 disp(['Vehicle ' num2str(v_id) ' changes target to ' num2str(vehicle.target)])
                            else
                                vehicle.target = 0; % Remove target
                                vehicle.messages(v_idx) = 4; % Notify old target to leave platoon
                            end
                        end
                end
                vehicles(v_idx).messages(v_id) = 0; % reset value
            end
        end
        if vehicle.target ~= 0 % Do checks if conn is still established and if targets are correct
            vehicle.isLeader = false;
            % Check if conn established
            if vehicles(vehicle.target).parameters.conn == false % Conn disabled
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
        
        min_idx = -1; % Keeps track of best vehicle
        % Get target
        if time > 7% Wait 7 seconds, avoids crashes at start of random tests
            %Init var for choosing targets
            min_diff_d_vel = 1000;
            min_diff_pos = 1000;
            for v_idx = 1:num_vehicles %
                if v_idx == v_id || vehicles(v_idx).parameters.conn == false
                    continue; % Skip ourselfs or for those with no conn
                end
                diff_pose = vehicles(v_idx).pose(1) - vehicle.pose(1); % Difference in meters between vehicles (x-axis)
                %         diff_vel = vehicle.velocity(1:2) - vehicles(v_idx).velocity(1:2);
                diff_d_vel = vehicles(v_idx).parameters.desired_vel - vehicle.parameters.desired_vel;
                
                % Save vehicle with lowest relative velocity within a
                % certain range
                % if (diff_pose < min_diff_pos) && (diff_pose < vehicle.parameters.max_range) && (abs(diff_d_vel) <=vehicle.parameters.vel_tresh/3.6) &&...
                if (abs(diff_d_vel) < min_diff_d_vel) && (diff_pose < vehicle.parameters.max_range) && (abs(diff_d_vel) <=vehicle.parameters.vel_tresh/3.6) &&...
                        (diff_pose >= 0)

                    % Make sure a vehicle doesnt pick a target while it's
                    % followers are in the way.
                    if vehicles(v_idx).target == 0
                        hold_num = sum(vehicles(v_idx).platoon_members); % Get how many members are in the platoon
                    else
                        hold_num = sum(vehicles(vehicles(v_idx).target).platoon_members); % Get how many members are in the platoon
                    end
                    if ~(diff_pose < 15*hold_num && ~isInSameLane(vehicle,vehicles(v_idx)))% hold distance 15*hold_num
                        if vehicles(v_idx).target ~= 0 % if that vehicle already has a target, check if vel within range
                            diff_d_vel_target = vehicles(vehicles(v_idx).target).parameters.desired_vel - vehicle.parameters.desired_vel;
                            if abs(diff_d_vel_target) <=vehicle.parameters.vel_tresh/3.6  
                                min_diff_d_vel = diff_d_vel;
                                min_idx = v_idx;
    %                             min_diff_pos = diff_pose;
    %                             min_idx = v_idx;
                            end
                        else
                            min_diff_d_vel = diff_d_vel;
                            min_idx = v_idx;
    %                         min_diff_pos = diff_pose;
    %                         min_idx = v_idx;
                        end
                    end
                end
            end
        end
        %Update
        if  min_idx ~= -1
            if vehicle.timings.timeGotTarget == 0 % Reset timer
                vehicle.timings.timeGotTarget = time; % Start timer to measure how long it takes to reach target
            end
            if vehicle.target ~= 0 % Already have a target
                vehicle.messages(vehicle.target) = 4; % notify old target that i leave
            end
            if vehicles(min_idx).target == 0 % New target doesn't have a target
                vehicle.target = min_idx; % Update our target.
            else % New target have a target, change to that one.
                vehicle.target = vehicles(min_idx).target; % Update to that vehicle.
            end
            vehicle.messages(vehicle.target) = 3; % notify target
%             disp(['Vehicle ' num2str(v_id) ' targets ' num2str(min_idx)])
        end
        
        % Follow target
        if vehicle.target ~= 0
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
            % Change lane
            if vehicle.parameters.lane ~= vehicles(idx).parameters.lane &&...
                    vehicles(idx).pose(1)-vehicle.pose(1) < 60 && ~vehicle.lane_keeping_var.isChangingLane % Change lane to target if within <60m
                [vehicle, changeLane] = change_lane(vehicle,vehicles(idx).parameters.lane); % Try to change lane
                if ~changeLane
                    % If lane change fails, other car propably in the way,
                    vehicle.messages(vehicle.target) = 4; % Notify old target to leave platoon
                    vehicle.target = 0; % Remove target
                else
%                    disp(['Vehicle ' num2str(v_id) ' changes lane to target lane']) 
                end
            end
            % Check if vehicle is at target
            if vehicle.target ~= 0 && vehicle.timings.timeGotTarget ~=0 && vehicle.timings.timeToTarget == 0
                time_headway = (vehicles(idx).pose(1) - vehicle.pose(1))/vehicle.velocity(1);
                if time_headway < vehicle.trailing_var.t_hw_conn+2.5 &&  time_headway > 0 && isInSameLane(vehicle,vehicles(vehicle.target))
                    vehicle.timings.timeToTarget = time - vehicle.timings.timeGotTarget;
                    if vehicle.timings.timeToTarget <= 2*vehicle.parameters.sample_time % Don't save if timing is very low,
                        % occurs when vehicles are teleported.
                        vehicle.timings.timeToTarget = 0;
                        vehicle.timings.timeGotTarget = 0;
                    else
%                         disp(['Vehicle ' num2str(v_id) ' time to target: ' num2str(vehicle.timings.timeToTarget)])
                    end
                end
            end
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

                    if vehicle.detections(idx_curr,1) < 50 && abs(vehicle.detections(idx_curr,2)) < pi/64 % Check if vehicle is in front
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
                if vehicle.detections(idx_curr,1) < 50 && abs(vehicle.detections(idx_curr,2)) < pi/64 % Check if vehicle is in front
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
        if abs(vehicle.detections(idx_curr,2)) > pi/2 % If idx behind, flip direction
            ttc = -ttc;
        end
        if vehicle.lane_keeping_var.isChangingLane && isInSameLane(vehicle,vehicles(idx)) && ttc < 5 && ttc > 0
            % Cancel lane change manouver if ttc low
            vehicle.parameters.lane = mod(vehicle.parameters.lane,2)+1;
            vehicle.lastChangeTry = time;
%             disp(['Vehicle ' num2str(v_id) ' canceled manouver due to low ttc'])
        end
        if abs(vehicle.detections(idx_curr,2)) < pi/16 % Check if vehicle is in front
            if ((ttc < 4 && ttc > 0) || vehicle.detections(idx_curr,1) < 6) && isInSameLane(vehicle,vehicles(idx))                        
                    velocity_proceeding_veh = (vehicle.detections_prev(idx_prev,1)- vehicle.detections(idx_curr,1))/vehicle.parameters.sample_time;
                    a = 0.23*(vehicle.detections(idx_curr,1)-vehicle.trailing_var.t_hw*vehicle.velocity(1)) + ...
                        0.07*(velocity_proceeding_veh);

                    vx = vehicle.velocity(1) + a*vehicle.parameters.sample_time; % Calculate velocity
                    if ttc < 4 && ttc > 0 && (vehicle.target ==0 || (vehicle.target ~=0 ...
                        && vehicles(vehicle.target).platoon_members(idx) == false)) && ~vehicle.lane_keeping_var.isChangingLane
                        [vehicle, changeLane] = change_lane(vehicle, mod(vehicle.parameters.lane,2)+1); % Try to change lane
                        if ~changeLane
%                             disp(['Vehicle ' num2str(v_id) ' changes lane to ' num2str(vehicle.parameters.lane) '.  Vehicle ' num2str(idx) ' is too close' ])
                        end
                    end                   
            elseif ttc < 5 && ttc > 0 && isInSameLane(vehicle,vehicles(idx))
                if vehicle.parameters.conn
                    % Connection
                    if vehicles(idx).parameters.conn && vehicle.target ~= 0 &&...
                            vehicles(vehicle.target).platoon_members(idx) == false &&...
                            time < time_out + vehicle.lastDecline % Make sure it is not in the same platoon
%                         disp(['Vehicle ' num2str(v_id) ' ask ' num2str(idx) ' to change lane'])
                        vehicle.messages(idx) = 1;
                    elseif vehicles(idx).parameters.conn && vehicle.isLeader &&...
                            time > time_out + vehicle.lastDecline % If leader ask vehicle to change lane 
%                         disp(['Vehicle ' num2str(v_id) ' ask ' num2str(idx) ' to change lane'])
                        vehicle.messages(idx) = 1;
                    end
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