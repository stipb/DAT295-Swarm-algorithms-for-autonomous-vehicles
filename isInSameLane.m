function val = isInSameLane(vehicle1,vehicle2)
%ISINSAMELANE Summary of this function goes here
%   Detailed explanation goes here
val = vehicle1.parameters.lane == vehicle2.parameters.lane;
end

