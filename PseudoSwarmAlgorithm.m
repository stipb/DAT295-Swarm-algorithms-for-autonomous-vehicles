clc;
clear;
close all;

%% Initialization

for each car i in S do
    for each dimension d in D do
        % Initialize all cars' position and velocity
        x(i,d) = Rnd(x_min, x_max)
        v(i,d) = Rnd((-v_max)/3, v_max/3)
    end for
        % Initialize cars' best position
        cb_i = x_i
        % Update the global best position
        if f(cb_i) < f(gb) then
            gb = cb_i
        end if
end for
    
%% Particle Swarm Optimization

% Initialize all cars
Initialize
repeat 
    for each car i in S do
        % Update the car's best position
        if f(x_i) < f(cb_i) then
            cb_i = x_i
        end if
        % Update the global best position
        if f(cb_i) < f(gb) then
            gb = cb_i
        end if
    end for
        % Update car's velocity and position
        for each car i in S do
            for each dimension d in D do
                v(i,d) = v(i,d)+C_1*Rnd(0,1)*[cb(i,d)-x(i,d)]+C_2*Rnd(0,1)*[gb_d-x(i,d)]
                x(i,d) = x(i,d)+v(i,d)
            end for
        end for
        % Advance iteration
        it = it+1
until it > MAX_ITERATIONS
                
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
           