close all;
clearvars -except f;
clc;

load('matlab.mat')

M = readmatrix('scope_0.csv');
M = M(4:end, :);
t = M(:, 1);
Y = M(:, 2);
V2 = M(:, 3);
% ZEROES T0
t = min_to_zero(t);

figure
plot(t, Y, t, V2)
legend("Resposta","Ruido")
title("Raw Data")
xlabel("Tempo[s]")
ylabel("Tensão [V]")

[i0, i1] = value_range_index(Y, 0.509, 3.2071);

% CUT
Y = Y(i0:i1);
t = t(i0:i1);

% NORMALIZE
Y = min_to_zero(Y);
t = min_to_zero(t);
y = v_to_rad(Y);


figure
plot(t, y)
title("Resposta cortada em RAD")
xlabel("Tempo[s]")
ylabel("Ângulo [Rad]")

y1 = min_to_zero(y);

figure
hold on
plot(t, y1)
plot(f)
title("Resposta cortada e normalizada em RAD")
xlabel("Tempo[s]")
ylabel("Ângulo [Rad]")
legend("Resposta", "Fitted")

%%
input = 15;                         % PWM[%]
N = 0.0002*input^2 + 0.0067*input;  % PwM[%] para F[N]
H = tf(f.a*f.d/N,[1, f.d, 0])       % theta[rad]/F[N]
[YY, TT] = step(H*N, t);

figure
hold on
plot(t, y1)
plot(TT, YY)
xlim([0 1.5])
title("Resposta cortada e normalizada em RAD")
xlabel("Tempo[s]")
ylabel("Ângulo [Rad]")
legend("Resposta", "FT")




function [i0, i1] = value_range_index(vec, v0, v1)
    i0 = find(vec > v0, 1);
    i1 = find(vec > v1, 1);
end

function v = min_to_zero(vec)
    v = vec - min(vec);
end

function x = v_to_rad(r)
    x = 0.6225*r - 1.1874;
end

function y = plant(t, A, p)
    y = A/p.*t - A/p^2 + A*exp(1)^(-p.*t)/p^2;
end
    