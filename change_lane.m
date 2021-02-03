function vehicle = change_lane(vehicle, target_lane,time)
%CHANGE_LANE Checks for vehicles before changing lane
changeLane = true;
if ~isempty(vehicle.detections)
    for i=1:length(vehicle.detections(:,2))
        if (abs(sin(vehicle.detections(i,2))) > 0.25 && vehicle.detections(i,1) < 7) || time < 2 + vehicle.lastChangeTry % Check if another vehicle is close
            changeLane = false;
        end
    end
    if changeLane
        vehicle.parameters.lane = target_lane; % Change to target lane
        vehicle.lane_keeping_var.isChangingLane = true;
    else
%         vehicle.trailing_var.brake = true;
        disp('Cancelled manouver')
    end
else
    vehicle.parameters.lane = target_lane; % Change to target lane
    vehicle.lane_keeping_var.isChangingLane = true;
end
end