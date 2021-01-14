function vehicle = change_lane(vehicle, target_lane)
%CHANGE_LANE Checks for vehicles before changing lane
changeLane = true;
check_mat = [1/2 5]; % Check matrix, sin(angle) < 1/2 and range < 10
if ~isempty(vehicle.detections)
    detected_angles = vehicle.detections(:,2);
    detected_angles_true = detected_angles;
    for i=1:length(detected_angles) % Fold angles larger than pi/2 to the right side
        if abs(detected_angles(i)) > pi/2
           detected_angles_true(i) = detected_angles(i) + pi/2;
        end
        if abs(sin(detected_angles(i))) > 0.5 && vehicle.detections(i,1) < 5
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