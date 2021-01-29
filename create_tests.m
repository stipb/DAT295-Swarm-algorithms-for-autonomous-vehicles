%% Create tests with this script
clear all
test_name = 'test_case_crash';
% - Init -
num_vehicles = 11;
sample_time = 0.05; % Time step [s]
sim_length = 100; % Simulation time [s]

% Set initial settnings for each vehicle:
init_x_position = [325.518821885936,159.306333349492,271.912073276339,456.684719175856,33.7217166714185,10.5484707869113,413.177195428673,249.842766049155,426.676127378040,296.400568809999,127.826569423917];
init_y_position = [24.7540352969461,23.9849124569386,24.5832889144960,25.8893940573100,23.3369882092586,25.1596203753985,26.8968865249549,24.3227282639611,23.2585343642035,26.2745629403288,26.5914627393612];

init_vel = [17.1919469328649,15.8710169600455,16.6407066601574,16.6677742411436,13.3874766434682,12.4357318420531,19.0394226037803,17.0833583110194,17.1692354962701,23.4199310980918,19.3522480609182]; % Initial velocity for vehicles
init_ang = [-0.0751946446522755,-0.0244077912330841,0.00301734643632373,0.0651627850984710,-0.0345143379407278,-0.0694066456346387,0.0975563115026982,-0.00160883834274084,0.0641335536524731,0.0846485701119370,0.000753998307452555]; % Starting angle for vehicles

lane = [2,2,2,1,1,2,2,1,1,1,2]; % Which lane the vehicles should drive in
init_conn = [true,true,true,true,true,true,true,true,true,true,true]; % Set if vehicles should have communcation enabled
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