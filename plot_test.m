%% Plots test data
clear all, close all

load('test_data/copy')
%%
% meanThroughput_conn = [];
% meanFuelConsumptionTot_conn = [];
% meanFuelConsumptionPVeh_conn = [];
% meanTimeToTarget_conn = [];
% 
% meanThroughput_noConn = [];
% meanFuelConsumptionTot_noConn = [];
% meanFuelConsumptionPVeh_noConn = [];

fn = fieldnames(data);
nmr_cases = numel(fn);
nmr_tests = numel(fieldnames(data.(fn{1})));
Throughput_conn = zeros(nmr_tests,nmr_cases);
FuelConsumptionTot_conn = zeros(nmr_tests,nmr_cases);
FuelConsumptionPVeh_conn = zeros(nmr_tests,nmr_cases);
TimeToTarget_conn = zeros(nmr_tests,nmr_cases);

Throughput_noConn  = zeros(nmr_tests,nmr_cases);
FuelConsumptionTot_noConn  = zeros(nmr_tests,nmr_cases);
FuelConsumptionPVeh_noConn  = zeros(nmr_tests,nmr_cases);

num_vehicles = zeros(nmr_tests,nmr_cases);
crash = 0;
%Extract data
for i=1:nmr_cases % cases
    fn_test = fieldnames(data.(fn{i}));
    for j=1:nmr_tests % test per case
        d_conn = data.(fn{i}).(fn_test{j}).conn;
        d_noConn = data.(fn{i}).(fn_test{j}).noConn;
        if d_conn.Throughput ~= 0
            Throughput_conn(j,i) = d_conn.Throughput*60; % [vehicles/min]
            FuelConsumptionTot_conn(j,i) = d_conn.FuelConsumptionTot/(0.7389*1000); %[l]
            FuelConsumptionPVeh_conn(j,i) = d_conn.FuelConsumptionPVeh/(0.7389*1000); %[l/vehicle]
            TimeToTarget_conn(j,i) = d_conn.MeanTimeToTarget; % [s]

            Throughput_noConn(j,i) = d_noConn.Throughput*60; % [vehicles/min]
            FuelConsumptionTot_noConn(j,i) = d_noConn.FuelConsumptionTot/(0.7389*1000); %[l]
            FuelConsumptionPVeh_noConn(j,i) =  d_noConn.FuelConsumptionPVeh/(0.7389*1000); %[l/vehicle]
        else
            crash = crash + 1;
             
        end
       num_vehicles(j,i) = d_conn.num_vehicles; 
    end
end
crash
%% calc mean and variance
Throughput_conn_mean = zeros(1,nmr_cases);
FuelConsumptionTot_conn_mean = zeros(1,nmr_cases);
FuelConsumptionPVeh_conn_mean = zeros(1,nmr_cases);
TimeToTarget_conn_mean = zeros(1,nmr_cases);
Throughput_noConn_mean = zeros(1,nmr_cases);
FuelConsumptionTot_noConn_mean = zeros(1,nmr_cases);
FuelConsumptionPVeh_noConn_mean = zeros(1,nmr_cases);

Throughput_conn_var = zeros(1,nmr_cases);
FuelConsumptionTot_conn_var = zeros(1,nmr_cases);
FuelConsumptionPVeh_conn_var = zeros(1,nmr_cases);
TimeToTarget_conn_var = zeros(1,nmr_cases);
Throughput_noConn_var = zeros(1,nmr_cases);
FuelConsumptionTot_noConn_var = zeros(1,nmr_cases);
FuelConsumptionPVeh_noConn_var = zeros(1,nmr_cases);

for i = 1:nmr_cases
    % --MEAN--
    % Conn
    Throughput_conn_mean(i) = mean(nonzeros(Throughput_conn(:,i)));
    FuelConsumptionTot_conn_mean(i) = mean(nonzeros(FuelConsumptionTot_conn(:,i)));
    FuelConsumptionPVeh_conn_mean(i) = mean(nonzeros(FuelConsumptionPVeh_conn(:,i)));
    TimeToTarget_conn_mean(i) = mean(nonzeros(TimeToTarget_conn(:,i)));
    % noConn
    Throughput_noConn_mean(i) = mean(nonzeros(Throughput_noConn(:,i)));
    FuelConsumptionTot_noConn_mean(i) = mean(nonzeros(FuelConsumptionTot_noConn(:,i)));
    FuelConsumptionPVeh_noConn_mean(i) = mean(nonzeros(FuelConsumptionPVeh_noConn(:,i)));
    
    % --VARIANCE--
    % Conn
    Throughput_conn_var(i) = var(nonzeros(Throughput_conn(:,i)));
    FuelConsumptionTot_conn_var(i) = var(nonzeros(FuelConsumptionTot_conn(:,i)));
    FuelConsumptionPVeh_conn_var(i) = var(nonzeros(FuelConsumptionPVeh_conn(:,i)));
    TimeToTarget_conn_var(i) = var(nonzeros(TimeToTarget_conn(:,i)));
    % noConn
    Throughput_noConn_var(i) = var(nonzeros(Throughput_noConn(:,i)));
    FuelConsumptionTot_noConn_var(i) = var(nonzeros(FuelConsumptionTot_noConn(:,i)));
    FuelConsumptionPVeh_noConn_var(i) = var(nonzeros(FuelConsumptionPVeh_noConn(:,i)));
end

%% plot

% Throughput
figure
errorbar(num_vehicles(1,:),Throughput_conn_mean,sqrt(Throughput_conn_var),'x'), hold on
errorbar(num_vehicles(1,:),Throughput_noConn_mean,sqrt(Throughput_noConn_var),'x')
xlabel('Number of vehicles'), ylabel('Throughput [vehicles/min]')
legend('Connection','No connection','Location','southeast')
ax = gca;
ax.XTick = num_vehicles(1,:);

% Fuel consumption
figure
errorbar(num_vehicles(1,:),FuelConsumptionTot_conn_mean,sqrt(FuelConsumptionTot_conn_var),'x'), hold on
errorbar(num_vehicles(1,:),FuelConsumptionTot_noConn_mean,sqrt(FuelConsumptionTot_noConn_var),'x')
xlabel('Number of vehicles'), ylabel('Total fuel consumption [L]')
legend('Connection','No connection','Location','southeast')
ax = gca;
ax.XTick = num_vehicles(1,:);

% Time to target
figure
errorbar(num_vehicles(1,:),TimeToTarget_conn_mean,sqrt(TimeToTarget_conn_var),'x')
xlabel('Number of vehicles'), ylabel('Time to target [s]')
ax = gca;
ax.XTick = num_vehicles(1,:);

