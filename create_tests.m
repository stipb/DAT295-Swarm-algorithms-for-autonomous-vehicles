%% Create tests with this script
clear all
test_name = 'test_case_test';
% - Init -
num_vehicles = 4;
sample_time = 0.05; % Time step [s]
sim_length = 40; % Simulation time [s]

% Set initial settnings for each vehicle:
init_x_position = [10 30 80 100];
init_y_position = [26 26 24 24];

init_vel = [66 80 75 60]/3.6; % Initial velocity for vehicles
init_ang = [0 0 0 0]; % Starting angle for vehicles

lane = [2,2,1,1]; % Which lane the vehicles should drive in
init_conn = [true,true,true,true]; % Set if vehicles should have communcation enabled
% init_conn = init_conn-1;
% Fill in these variables for events that happen during simulation
% leave variables as empties if no event should occur. 
% Break communication

%% FOR RANDOM TESTS:
% min_vel = 40;
% max_vel = 90;
% max_angle = pi/32;
% 
%         init_x_position = zeros(1,num_vehicles);
%         init_y_position = zeros(1,num_vehicles);
%         init_vel = zeros(1,num_vehicles);
%         init_ang = zeros(1,num_vehicles);
%         lane = zeros(1,num_vehicles);
%         init_conn = ones(1,num_vehicles);
%         for v_idx = 1:num_vehicles
%             % Random x position
%             init_x_position(v_idx) = (480-5).*rand(1,1) + 5;
%             % Random y position
%             init_y_position(v_idx) = (27-23).*rand(1,1) + 23;
%             % Random velocity
%             init_vel(v_idx) = ((max_vel-min_vel).*rand(1,1) + min_vel)/3.6;
%             % Random angle
%             init_ang(v_idx) = 2*max_angle.*rand(1,1) - max_angle;
%             % Random lane
%             lane(v_idx) = randi([1, 2], 1);
%         end

%% Code events
% _ids: what car
% _times: at what time (make sure it's possible with sample_time)
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

%% Do checks for variables
if num_vehicles ~= length(init_x_position) || num_vehicles ~= length(init_y_position) || num_vehicles ~= length(init_conn)... 
        || num_vehicles ~=length(init_vel) || num_vehicles ~= length(init_ang) || num_vehicles ~= length(lane)
   error('Make sure the number of vehicles matches the size of initial settnings arrays.') 
end
if length(bc_ids) ~= length(bc_times)
    error('The length of bc_ids and bc_times need to be the same size.')
end
if length(ec_ids) ~= length(ec_times)
    error('The length of ec_ids and ec_times need to be the same size.')
end
if length(cl_ids) ~= length(cl_times)
    error('The length of cl_ids and cl_times need to be the same size.')
end
if length(cv_ids) ~= length(cv_times) && length(cl_times) ~= length(cv_vel)
    error('The length of cl_ids and cl_times need to be the same size.')
end
save(['test_cases/' test_name])
disp(['Saved ' test_name])