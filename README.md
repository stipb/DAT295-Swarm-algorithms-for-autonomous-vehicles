# Swarm-algorithms-for-autonomous-vehicles

Swarm-algorithms-for-autonomous-vehicles includes a simulation tool for autonomous vehicles with swarm like behavoiur. 

Note: The Mobile Robotics Simulation Toolbox is used, see:
https://se.mathworks.com/matlabcentral/fileexchange/66586-mobile-robotics-simulation-toolbox
## Getting started
Tested on MATLAB 2020b.

1. Head over to the [toolbox](https://github.com/mathworks-robotics/mobile-robotics-simulation-toolbox) page and follow the instructions. 
2. Make sure you also have installed the requried toolboxes, see: https://se.mathworks.com/matlabcentral/fileexchange/66586-mobile-robotics-simulation-toolbox.
## Usage

[multicar_env.m](multicar_env.m) contains the simulation and swarm control. To run a certain add to line(5-6):
```matlab
clear all
load('test_cases/test_case_#')
  ```
To run automatic tests run the file [runRandomTests.m](runRandomTests.m). Before running it make sure the above lines are commented out or removed, otherwise it will run the same test over and over.

### Configure simulation

**Flags:**  
save_as_video = true; % Saves a recording of the test.  
save_data = true; % Saves data and runs fuel consumption algorithm.  
timed = true; % Try to match loop time with actual time.  

Too configure the vehicle parameters see line(76-95) in [multicar_env.m](multicar_env.m).

In [runRandomTests.m](runRandomTests.m) you can configure:
```matlab  
test_name = 'test_name'; % Test name.  
number_of_tests = #; % Number of tests per vehicle configuration.  
number_of_vehicles = [# # # #]; % specifies the set of vehicles that are gonne be tested, # is the number of vehicles.  
sample_time = #; % Specifies the time step.. 
sim_length = #; % Simulation length.  
min_vel = 40; % [km/h] minimum velocity.  
max_vel = 90; % [km/h] maximum velocity.   
max_angle = pi/64; % maximum angle.

% Break connection
% _ids: what vehicle
% _times: at what time (make sure it's possible with sample_time).
bc_ids = []; %
bc_times = [];

% Enable connection
ec_ids = [];
ec_times = [];

% Change lane
cl_ids = [];
cl_times = [];

% Change velocity
cv_ids = [];
cv_times = [];
cv_vel = [];
```
Some of these variables are also available in [create_tests.m](create_tests.m). A script that let you create new tests.

## Videos

In [video_files](/video_files/) you can find recorded videos for some test_cases in [test_cases](/test_cases/).
