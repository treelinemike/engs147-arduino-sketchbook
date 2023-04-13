% restart
close all; clear; clc;

tab = readtable('motor_mapping.csv');

% check pos PWM
tab_mask = tab.PWM > 0;
pwm = tab.PWM(tab_mask)
V = tab.V_ss(tab_mask)

figure; hold on; grid on;
plot(V,pwm,'.','MarkerSize',20,'Color',[0 0 0.8]);

a1 = 3.7;
b1 = -7.1;
c1 = 27.9;

a2 = 61.9;
b2 = 768.3;
c2 = 2525.8;



Vin = 0:0.1:9.6;
y1 = a1*Vin.^2+b1*Vin+c1;
y2 = a2*Vin.^2+b2*Vin+c2;

plot(Vin,y1,'-','LineWidth',1.6,'Color',[0.8 0 0]);
plot(Vin,y2,'-','LineWidth',1.6,'Color',[0.8 0 0]);
