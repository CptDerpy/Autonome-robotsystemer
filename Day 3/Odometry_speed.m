
%% 3.3 - Odemetry Speed VS time. 0.2m/s on 2 meters. Plot x,y,theta
clc
clear
F = importdata('data_log_2m_02.dat');
t = linspace(0, length(F(:,1)), length(F(:,1)));

figure(1)

subplot(3,1,1)
plot(t,F(:,2))
title('x-position 0.2m/s on 2m')
xlabel('Time (ms)')
ylabel('position (m)')
%xlim([0 500]);
%ylim([-0.35 0.35]);

subplot(3,1,2)
plot(t,F(:,3))
title('y-position 0.2m/s on 2m')
xlabel('Time (ms)')
ylabel('position (m)')
%xlim([0 500]);
%ylim([-0.35 0.35]);

subplot(3,1,3)
plot(t,F(:,4))
title('theta angle 0.2m/s on 2m')
xlabel('Time (ms)')
ylabel('Theta (radians)')
%xlim([0 500]);
%ylim([-0.35 0.35]);

%% 3.3 - Odemetry Speed VS time. 0.4m/s on 2 meters. Plot x,y,theta
clc
clear
F = importdata('data_log_2m_04.dat');
t = linspace(0, length(F(:,1)), length(F(:,1)));

figure(2)

subplot(3,1,1)
plot(t,F(:,2))
title('x-position 0.4m/s on 2m')
xlabel('Time (ms)')
ylabel('position (m)')
%xlim([0 500]);
%ylim([-0.35 0.35]);

subplot(3,1,2)
plot(t,F(:,3))
title('y-position 0.4m/s on 2m')
xlabel('Time (ms)')
ylabel('position (m)')
%xlim([0 500]);
%ylim([-0.35 0.35]);

subplot(3,1,3)
plot(t,F(:,4))
title('theta angle 0.4m/s on 2m')
xlabel('Time (ms)')
ylabel('Theta (radians)')
%xlim([0 500]);
%ylim([-0.35 0.35]);

%% 3.3 - Odemetry Speed VS time. 0.6m/s on 2 meters. Plot x,y,theta
clc
clear
F = importdata('data_log_2m_06.dat');
t = linspace(0, length(F(:,1)), length(F(:,1)));

figure(3)

subplot(3,1,1)
plot(t,F(:,2))
title('x-position 0.6m/s on 2m')
xlabel('Time (ms)')
ylabel('position (m)')
%xlim([0 500]);
%ylim([-0.35 0.35]);

subplot(3,1,2)
plot(t,F(:,3))
title('y-position 0.6m/s on 2m')
xlabel('Time (ms)')
ylabel('position (m)')
%xlim([0 500]);
%ylim([-0.35 0.35]);

subplot(3,1,3)
plot(t,F(:,4))
title('theta angle 0.6m/s on 2m')
xlabel('Time (ms)')
ylabel('Theta (radians)')
%xlim([0 500]);
%ylim([-0.35 0.35]);

%% Total plot of different speed settings.
clc
clear
close
A = importdata('data_log_2m_02.dat');
B = importdata('data_log_2m_04.dat');
C = importdata('data_log_2m_06.dat');

tA = linspace(0, length(A(:,1)), length(A(:,1)));
tB = linspace(0, length(B(:,1)), length(B(:,1)));
tC = linspace(0, length(C(:,1)), length(C(:,1)));

figure(4)
hold all
plot(tA,A(:,2),'DisplayName','0.2 m/s')
plot(tB,B(:,2),'DisplayName','0.4 m/s')
plot(tC,C(:,2),'DisplayName','0.6 m/s')
title('Velocity 0.2, 0.4 and 0.6 m/s on 2 meters')
xlabel('Time (ms)')
ylabel('position (m)')

legend('show')

print -painters -dpdf -r600 test.pdf
%legend('0.2 m/s','0.4 m/s', '0.6 m/s')



