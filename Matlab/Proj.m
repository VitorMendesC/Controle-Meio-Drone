clc;
clearvars -except C CzPID;
close all;

load('workspace.mat')
Ts = 20E-3;

s = tf('s');
G = 16.49/(s^2+0.4646*s);
G_alternativa = 14.69/(s^2+0.2016*s);
Gz = c2d(G, Ts);

Fz = feedback(CzPID*Gz, 1);

figure
step(Fz)




function rad = ADC_to_rad(ADC_value)
    rad = 0.0005*ADC_value - 1.1874;
end

function f_r = pwm_to_f_right(PWM)          %PWM em porcento [%]
    f_r = 0.0301*PWM - 0.3834;
end

function f_l = pwm_to_f_left(PWM)          %PWM em porcento [%]
    f_l = 0.0316*PWM - 0.3992;
end