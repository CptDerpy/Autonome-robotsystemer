%% Odometry dataplots 3.2 Clock wise AND Counter clock wise

% Dataset is generated from a simulation. x, y and theta are calculated
% from odemetric relations (see slides on odemetry).

%% 3.2 - x,y,theta VS time. Counter Clock Wise
clc
clear
F = importdata('data_log_ccw3_2.dat');
t = linspace(0, 1000, 1000);
t = t./100;

figure(1)

subplot(3,1,1)
plot(t,F(:,2))
title('SMR x-position (CCW)')
xlabel('Time (s)')
ylabel('position (m)')
%xlim([0 500]);
%ylim([-0.35 0.35]);

subplot(3,1,2)
plot(t,F(:,3))
title('SMR y-position (CCW)')
xlabel('Time (s)')
ylabel('position (m)')
%xlim([0 500]);
%ylim([-0.35 0.35]);

subplot(3,1,3)
plot(t,F(:,4))
title('SMR theta angle (CCW)')
xlabel('Time (s)')
ylabel('Theta (radians)')
%ytick([-2*pi -(3*pi/2) -pi -pi/2 0 pi/2 pi (3*pi)/2 2*pi])
%ytickLabel({'-2\pi','-3\pi/2','-\pi', '-\pi/2' ,'0','\pi/2','\pi','3\pi/2', '2\pi'})
%xlim([0 500]);
%ylim([-0.35 0.35]);

%% 3.2 - x,y,theta VS time. Clock Wise
clc
clear
F = importdata('data_log_cw3_2.dat');
t = linspace(0, 1000, 1000);

%set('figure(2)', 'WindowStyle', 'docked')
figure(2)

subplot(3,1,1)
plot(t,F(:,2))
title('SMR x-position (CW)')
xlabel('Time (s)')
ylabel('position (m)')
%xlim([0 500]);
%ylim([-0.35 0.35]);

subplot(3,1,2)
plot(t,F(:,3))
title('SMR y-position (CW)')
xlabel('Time (s)')
ylabel('position (m)')
%xlim([0 500]);
%ylim([-0.35 0.35]);

subplot(3,1,3)
plot(t,F(:,4))
title('SMR theta angle (CW)')
xlabel('Time (s)')
ylabel('Theta (radians)')
%xlim([0 500]);
%ylim([-0.35 0.35]);

