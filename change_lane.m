function vehicle = change_lane(vehicle, target_lane)
%CHANGE_LANE Checks for vehicles before changing lane
changeLane = true;
check_mat = [1/2 5]; % Check matrix, sin(angle) < 1/2 and range < 10
if ~isempty(vehicle.detections)
    detected_angles = vehicle.detections(:,2);
    for i=1:length(detected_angles) % Fold angles larger than pi/2 to the right side
        if abs(detected_angles(i)) > pi/2
           detected_angles(i) = detected_angles(i) + pi/2;
        end
        if all([detected_angles(i) vehicle.detections(i,1)] > check_mat)
            changeLane = false;
        end
    end
    if changeLane
        vehicle.parameters.lane = target_lane; % Change to target lane
    end
else
    vehicle.parameters.lane = target_lane; % Change to target lane
end
end

