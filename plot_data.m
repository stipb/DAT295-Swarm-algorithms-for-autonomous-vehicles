function plot_data(velocities,num_vehicles,time)
%% Plots all saved data
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
    plot(time,velocities(:,v_idx,1)*3.6,'Color',cmap(v_idx, :),'Linestyle',style); % plot
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
end