%% Create tests with this script
clear all
test_name = 'test_case_many';
% - Init -
num_vehicles = 10;
sample_time = 0.05; % Time step [s]
sim_length = 40; % Simulation time [s]

% Set initial settnings for each vehicle:
init_x_position = [10 50 60 80 150 180 210 250 260 300];
init_y_position = [24 26 26 26 24 24 26 24 24 26];

init_vel = [55 56 40 45 60 40 56 80 40 60]./3.6; % Initial velocity for vehicles
init_ang = [0 0 0 0 0 0 0 0 0 0]; % Starting angle for vehicles

lane = [1 2 1 2 1 2 2 1 2 2]; % Which lane the vehicles should drive in
init_conn = [true true true true true true true true true true]; % Set if vehicles should have communcation enabled
init_conn = init_conn-1;
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
cv_ids = [0];
cv_times = [0];
cv_vel = [0];
% Do checks for variables
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