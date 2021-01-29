%% Calculate fuel consumption

x_vel = velocities(:,:,1); % Get velocity in x-axis
acc = diff(x_vel); % Calculate acceleration
instFuelConsp = zeros(size(acc)); % init matrix
for v_idx = 1:num_vehicles
    for t_idx = 1:numel(time)-1
        instFuelConsp(t_idx,v_idx) = instFuelConsump(x_vel(1+t_idx,v_idx)*3.6,acc(t_idx,v_idx));
    end
end

figure
hold all
legend_str = [];
cmap = lines(num_vehicles);
for v_idx = 1:num_vehicles
    if mod(v_idx,2) == 1 % Change linestyle 
        style = '-';
    else
        style = '--';
    end
    plot(time(2:end),instFuelConsp(:,v_idx,1)*3.6,'Color',cmap(v_idx, :),'Linestyle',style); % plot
    % Make sure each row has the same size
    if v_idx < 10
        v_str = ['0' num2str(v_idx)];
    else
        v_str = num2str(v_idx);
    end
    % Save legend for each vehicle
    legend_str = [legend_str; 'Vehicle ' v_str];
end
legend(legend_str)
xlabel('Time [s]'), ylabel('Velocity [km/h]')
%%
function MOE_e = instFuelConsump(s,a)
% Coefficients
coeff_pos = [-0.87605 0.03627 -0.00045 2.55e-06;
            0.081221 0.009246 -0.00046 4.00e-06;
            0.037039 -0.00618 2.96e-04 -1.86e-06;
            -0.00255 0.000468 -1.79e-05 3.86e-08];
coeff_neg = [-0.75584 0.021283 -0.00013 7.39e-07;
            -0.00921 0.011364 -0.0002 8.45e-08;
            0.036223 0.000226 4.03e-08 -3.5e-08;
            0.003968 -9e-05 2.42e-06 -1.6e-08];
% Estimate fuel consumption
MOE_e = 0;
for i=1:4
    for j=1:4
        if a >= 0
            MOE_e = MOE_e + coeff_pos(i,j)*s^i*a^j;
        else
            MOE_e = MOE_e + coeff_neg(i,j)*s^i*a^j;
        end
    end
end
MOE_e = exp(MOE_e);
end