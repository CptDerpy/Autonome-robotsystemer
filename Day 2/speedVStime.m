%% Basic motion control 3.1 Plots the motorspeeds versus time.
clc
clear
F = importdata('data_log2.dat');
t = linspace(0, 500, 500);
subplot(2,1,1)
plot(t,F(:,2))
xlim([0 500]);
ylim([-0.35 0.35]);
subplot(2,1,2)
plot(t,F(:,3))
xlim([0 500]);
ylim([-0.35 0.35]);