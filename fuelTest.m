%% Test fuel consp
close all
a = 1.8;

v = 0:0.5:140;
c = 1;
figure
for i=1:length(v)
    MOE_e = instFuelConsump(v(i)*c,a*1)/(0.7389*1000);
    plot(v(i),MOE_e,'r.'), hold on
end
a = 0.9;
for i=1:length(v)
    MOE_e = instFuelConsump(v(i)*c,a*1)/(0.7389*1000);
    plot(v(i),MOE_e,'b.'), hold on
end
a = 0.0;
for i=1:length(v)
    MOE_e = instFuelConsump(v(i)*c,a*1)/(0.7389*1000);
    plot(v(i),MOE_e,'m.'), hold on
end
v = 0:0.5:40;
a = -1:0.062:4;
MOE_e = zeros(length(v),length(v));
for i=1:length(v)
    for j=1:length(a)
        MOE_e(i,j) = instFuelConsump(v(i)/3.6,a(j))/(0.7389*1000);
    end
end
figure
plot3(a,v,MOE_e)
ylim([0 0.012])
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