%% 3.1 Matlab script to calculate Eb and Ed.
clear
clc
Xcw = -0.05;
Ycw = -0.05;
Xccw = -0.05;
Yccw = 0.05;
L = 3;
b0 = 0.26;

% The calculation of Eb
alphaX = ((Xcw + Xccw) / (-4*L)) * (180/pi);
alphaY = ((Ycw - Yccw) / (-4*L)) * (180/pi);
alpha = (alphaX + alphaY) / 2;

Eb = 90/(90-alpha)

% The calculation of Ed
betaX = ((Xcw - Xccw) / (-4*L)) * (180/pi);
betaY = ((Ycw + Yccw) / (-4*L)) * (180/pi);
beta = (betaX + betaY) / 2;

R = (L/2) / sin(degtorad(beta/2));

Ed = (R + b0/2) / (R - b0/2)

res = Eb * 0.26