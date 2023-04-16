% restart
close all; clear; clc;

tab = readtable('test.csv');

t = tab.Var1/1e6;
theta = tab.Var2;
omega = tab.Var3;
err = tab.Var4;
V = tab.Var5;
pwm = tab.Var6;

figure;
ax(1) = subplot(2,1,1);
hold on; grid on
plot(t,omega);
ax(2) = subplot(2,1,2);
hold on; grid on;
plot(t,pwm);

linkaxes(ax,'x');