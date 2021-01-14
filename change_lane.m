function vehicle = change_lane(vehicle, target_lane)
%CHANGE_LANE Checks for vehicles before changing lane
changeLane = true;
if ~isempty(vehicle.detections)
    for i=1:length(detected_angles) % Fold angles larger than pi/2 to the right side
        if abs(sin(detected_angles(i))) > 0.5 && vehicle.detections(i,1) < 5 % Check if vehicle is close in another lane
            changeLane = false;
        end
    end
    if changeLane
        vehicle.parameters.lane = target_lane; % Change to target lane
        vehicle.lane_keeping_var.isChangingLane = true;
    else
        disp('Cancelled manouver')
    end
else
    vehicle.parameters.lane = target_lane; % Change to target lane
    vehicle.lane_keeping_var.isChangingLane = true;
end
end