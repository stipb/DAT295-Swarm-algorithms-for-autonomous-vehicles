%% Create tests with this script
clear all
test_name = 'test_case_8v';
% - Init -
num_vehicles = 8;
sample_time = 0.05; % Time step [s]
sim_length = 100; % Simulation time [s]

% Set initial settnings for each vehicle:
init_x_position = [10*2 20*2 30*2 40*2 50*2 60*2 70*2 80*2 90*2 100*2 110*2 120*2 130*2 140*2 150*2];
init_y_position = [24 24 24 24 24.7540352969461,23.9849124569386,24.5832889144960,25.8893940573100,23.3369882092586,25.1596203753985,26.8968865249549,24.3227282639611,23.2585343642035,26.2745629403288,26.5914627393612];

init_vel = [15.8710169600455,16.6407066601574,16.6677742411436,13.3874766434682,17.1919469328649,15.8710169600455,16.6407066601574,16.6677742411436,13.3874766434682,12.4357318420531,19.0394226037803,17.0833583110194,17.1692354962701,23.4199310980918,19.3522480609182]; % Initial velocity for vehicles
init_ang = [-0.0751946446522755,-0.0244077912330841,0.00301734643632373,0.0651627850984710,-0.0751946446522755,-0.0244077912330841,0.00301734643632373,0.0651627850984710,-0.0345143379407278,-0.0694066456346387,0.0975563115026982,-0.00160883834274084,0.0641335536524731,0.0846485701119370,0.000753998307452555]; % Starting angle for vehicles

lane = [1,2,1,1,2,2,2,1,1,2,2,1,1,1,2]; % Which lane the vehicles should drive in
% init_conn = [true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true]; % Set if vehicles should have communcation enabled
% init_conn = init_conn-1;
% Fill in these variables for events that happen during simulation
% leave variables as empties if no event should occur. 
% Break communication
min_vel = 40;
max_vel = 90;
max_angle = pi/32;

        init_x_position = zeros(1,num_vehicles);
        init_y_position = zeros(1,num_vehicles);
        init_vel = zeros(1,num_vehicles);
        init_ang = zeros(1,num_vehicles);
        lane = zeros(1,num_vehicles);
        init_conn = ones(1,num_vehicles);
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