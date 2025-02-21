function w = lane_keeper(vehicle)
switch vehicle.parameters.lane
    case 1
        range = cos(vehicle.pose(3))*vehicle.ranges(2);
        range_prev = cos(vehicle.pose_prev(3))*vehicle.ranges_prev(2);
        range_d = diff([range range_prev]);
        err_right = range-vehicle.lane_keeping_var.dist;
        w = -0.10*err_right + 1.5*range_d;
    case 2
        range = cos(vehicle.pose(3))*vehicle.ranges(1);
        range_prev = cos(vehicle.pose_prev(3))*vehicle.ranges_prev(1);
        range_d = diff([range range_prev]);
        err_left = range-vehicle.lane_keeping_var.dist;
        w = 0.10*err_left + -1.5*range_d;
end
end