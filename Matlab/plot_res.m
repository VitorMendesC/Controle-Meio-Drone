clc;
clear;
close all;

TS = 20E-3;

dt = readmatrix("route.txt");
dr = dt(:, 2);
tr = dt(:, 1);

tr3 = tr + 5;
tr3(1) = 0;

figure
plot(tr3, dr);
xlabel("Tempo [s]")
ylabel("\Theta [rad]")
title("Trajetória planejada")
xlim([0 35])
ylim([-0.71 0.71])

%% Sem disturbio
data = readmatrix("scope_23.csv");
t1 = data(:, 1);
d1 = data(:, 2);
%t1 = t1 - min(t1);

i0 = find(t1 > -36, 1);
t1 = t1(i0:end);
d1 = d1(i0:end);

t1 = t1 - min(t1);

d1 = V_to_rad(d1);
tr1 = tr + 4.7;
tr1(1) = 0;


figure
plot(t1, d1, tr1, dr)
xlabel("Tempo [s]")
ylabel("\Theta [rad]")
title("Resposta a trajetória")
legend("Experimental","Teórico")
xlim([0 35])
ylim([-0.71 0.71])

%% Com disturbio

data = readmatrix("scope_24.csv");
t1 = data(:, 1);
d1 = data(:, 2);
t1 = t1 - min(t1);

i0 = find(t1 > 9.5, 1);
t1 = t1(i0:end);
d1 = d1(i0:end);

t1 = t1 - min(t1);

d1 = V_to_rad(d1);
tr2 = tr + 5.5;
tr2(1) = 0;

figure
plot(t1, d1, tr2, dr)
xlabel("Tempo [s]")
ylabel("\Theta [rad]")
title("Resposta a trajetória com disturbio")
legend("Experimental","Teórico")
xlim([0 40])
ylim([-0.71 0.71])

function rad = V_to_rad(v)
    rad = 0.6225*v - 1.1874;
end

