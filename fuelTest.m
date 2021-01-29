%% Test fuel consp
a = 1.8;

v = 0:0.1:140;

figure
for i=1:length(v)
    MOE_e = instFuelConsump(v(i),a);
    plot(v(i),MOE_e,'r.'), hold on
end
a = 0.9;
for i=1:length(v)
    MOE_e = instFuelConsump(v(i),a);
    plot(v(i),MOE_e,'b.'), hold on
end

function MOE_e = instFuelConsump(s,a)
% Coefficients
coeff_pos = [-0.87605 0.03627 -0.00045 2.55e-06;
            0.081221 0.009246 -0.00046 4.00e-06;
            0.037039 -0.00618 2.96e-04 -1.86e-06;
            -0.00255 0.000468 -1.79e-05 3.86e-08];
coeff_neg = [-0.75584 0.021283 -0.00013 7.39e-07;
            -0.00921 0.011364 -0.0002 8.45e-07;
            0.036223 0.000226 4.03e-08 -3.5e-08;
            0.003968 -9e-05 2.42e-06 -1.6e-08];
% Estimate fuel consumption
MOE_e = 0;
for i=1:4
    for j=1:4
        if a >= 0
            MOE_e = MOE_e + coeff_pos(j,i)*s^(j-1)*a^(i-1);
        else
            MOE_e = MOE_e + coeff_neg(j,i)*s^(j-1)*a^(i-1);
        end
    end
end
MOE_e = exp(MOE_e);
end