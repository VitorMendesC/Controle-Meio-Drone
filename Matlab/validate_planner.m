clc;
clear;
close all;

data = readmatrix("route.txt");
t = data(:, 1);
d = data(:, 2);

figure
plot(t,d)
xlabel("Tempo [s]")
ylabel("\Theta [rad]")

xlim([0 10])