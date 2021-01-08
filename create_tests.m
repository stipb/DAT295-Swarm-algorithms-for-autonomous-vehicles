%% Create tests with this script

test_name = 'test_case1';
% - Init -
num_vehicles = 4;
sample_time = 0.1; % Time step [s]
sim_length = 60; % Simulation time [s]

% Set initial settnings for each vehicle:
init_x_position = [100 200 300 400];
init_y_position = [25 25 25 25];

init_vel = [11 12 23 27]./3.6; % Initial velocity for vehicles
init_ang = [-pi/16 -pi/50 -pi/16 -pi/16]; % Starting angle for vehicles

lane = [2 2 1 1]; % Which lane the vehicles should drive in
init_conn = [true true true true]; % Set if vehicles should have communcation enabled

% Fill in these variables for events that happen during simulation
% leave variables as empties if no event should occur. 

% Break communication
bc_ids = [1 3];
bc_times = [140 140];

% Enable communication
ec_ids = [1 3];
ec_times = [280 280];

% Change lane
cl_ids = [3];
cl_times = [90];

% Do checks for variables
if num_vehicles ~= length(init_x_position) || num_vehicles ~= length(init_y_position)... 
        || num_vehicles ~=length(init_vel) || num_vehicles ~= length(init_ang) || num_vehicles ~= length(lane)
   error('Make sure  the number of vehicles matches the size of initial settnings arrays.') 
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
save(test_name)
disp(['Saved ' test_name])