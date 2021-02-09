%% Run random tests continuesly and save values
clear all, close all
test_name = 'testGroup_final';
number_of_tests = 1;
number_of_vehicles = [13 14 15];
sample_time = 0.05; % Time step [s]
sim_length = 80; % Simulation time [s]
% number_of_tests = 10;
% number_of_vehicles = [4 5 6 7 8 9 10 11 12 13 14 15];
% sample_time = 0.05; % Time step [s]
% sim_length = 80; % Simulation time [s]
min_vel = 40; %[km/h]
max_vel = 90; %[km/h]
max_angle = pi/32;

% init_conn = init_conn-1;
% Fill in these variables for events that happen during simulation
% leave variables as empties if no event should occur. 

% Break communication
bc_ids = [];
bc_times = [];

% Enable communication
ec_ids = [];
ec_times = [];

% Change lane
cl_ids = [];
cl_times = [];

% Change velocity
cv_ids = [];
cv_times = [];
cv_vel = [];

%% Init data structure
data = struct();
for test = 1:length(number_of_vehicles)
    num_vehicles = number_of_vehicles(test);
    name = ['Tests_with_' num2str(num_vehicles) '_vehicles'];
    for test_iteration = 1:number_of_tests
        data.(name).(['Test_' num2str(test_iteration)]) = [];
    end
end
%% Run simulations
for test = 1:length(number_of_vehicles)
    num_vehicles = number_of_vehicles(test);
    for test_iteration = 1:number_of_tests
        init_x_position = zeros(1,num_vehicles);
        init_y_position = zeros(1,num_vehicles);
        init_vel = zeros(1,num_vehicles);
        init_ang = zeros(1,num_vehicles);
        lane = zeros(1,num_vehicles);
        for v_idx = 1:num_vehicles
            % Random x position
            init_x_position(v_idx) = (480-5).*rand(1,1) + 5;
            % Random y position
            init_y_position(v_idx) = (27-23).*rand(1,1) + 23;
            % Random velocity
            init_vel(v_idx) = ((max_vel-min_vel).*rand(1,1) + min_vel)/3.6;
            % Random angle
            init_ang(v_idx) = 2*max_angle.*rand(1,1) - max_angle;
            % Random lane
            lane(v_idx) = randi([1, 2], 1);
        end
        % Run simulation (conn)
        init_conn = true(1,num_vehicles);
        try
            multicar_env
            % Save data
            data.(['Tests_with_' num2str(num_vehicles) '_vehicles']).(['Test_' num2str(test_iteration)]).conn= ...
                struct('Throughput',throughput,'FuelConsumptionTot',totalFuel,'FuelConsumptionPVeh',...
                totalFuel/num_vehicles,'MeanTimeToTarget',mean(timeToTarget),'Velocities',velocities,'num_vehicles',num_vehicles);
        catch ex
            disp('---Test crashed---')
            data.(['Tests_with_' num2str(num_vehicles) '_vehicles']).(['Test_' num2str(test_iteration)]).conn= ...
                struct('Throughput',0,'FuelConsumptionTot',0,'FuelConsumptionPVeh',...
                0,'MeanTimeToTarget',0,'Velocities',[],'num_vehicles',num_vehicles);
        end
        % Run simulation (no conn)
        init_conn = false(1,num_vehicles);
        try
            multicar_env
            % Save data
            data.(['Tests_with_' num2str(num_vehicles) '_vehicles']).(['Test_' num2str(test_iteration)]).noConn= ...
                struct('Throughput',throughput,'FuelConsumptionTot',totalFuel,'FuelConsumptionPVeh',...
                totalFuel/num_vehicles,'MeanTimeToTarget',mean(timeToTarget),'Velocities',velocities,'num_vehicles',num_vehicles);
        catch ex
            disp('---Test crashed---')
            data.(['Tests_with_' num2str(num_vehicles) '_vehicles']).(['Test_' num2str(test_iteration)]).noConn= ...
                struct('Throughput',0,'FuelConsumptionTot',0,'FuelConsumptionPVeh',...
                0,'MeanTimeToTarget',0,'Velocities',[],'num_vehicles',num_vehicles);
        end
    end
end
save(['test_data/' test_name],'data')