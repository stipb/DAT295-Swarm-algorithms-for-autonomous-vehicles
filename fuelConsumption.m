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
    plot(time(2:end),instFuelConsp(:,v_idx,1),'Color',cmap(v_idx, :),'Linestyle',style); % plot
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
xlabel('Time [s]'), ylabel('Fuel consumption')
%%
function tmp = instFuelConsump(s,a)
% Coefficients
coeff_pos = [-0.87605 0.03627 -0.00045 2.55e-6;
            0.081221 0.009246 -0.00046 4.00e-6;
            0.037039 -0.00618 2.96e-4 -1.86e-6;
            -0.00255 0.000468 -1.79e-5 3.86e-8];
coeff_neg = [-0.75584 0.021283 -0.00013 7.39e-7;
            -0.00921 0.011364 -0.0002 8.45e-7;
            0.036223 0.000226 4.03e-8 -3.5e-8;
            0.003968 -9e-5 2.42e-6 -1.6e-8];
% Estimate fuel consumption
tmp = 0;
for i=1:4
    for j=1:4
        if a >= 0
            tmp = tmp + coeff_pos(j,i)*s^(i-1)*a^(j-1);
        else
            tmp = tmp + coeff_neg(j,i)*s^(i-1)*a^(j-1);
        end
    end
end
tmp = exp(tmp);
end