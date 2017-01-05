%% Basic motion control 3.1 Plots the motorspeeds versus time.
clc
clear
F = importdata('data_log.dat');
t = linspace(0, length(F(:,1)), length(F(:,1)));
t = t./100;

figure(1)

subplot(2,1,1)
plot(t,F(:,2))
title('x')
ylim([-0.35 0.35]);
xlabel('Time (s)')
ylabel('Velocity (m/s)')

subplot(2,1,2)
plot(t,F(:,3))
title('y')
ylim([-0.35 0.35]);
xlabel('Time (s)')
ylabel('Velocity (m/s)')
