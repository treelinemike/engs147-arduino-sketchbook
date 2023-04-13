% restart
close all; clear; clc;

tab = readtable('test.csv');

t = tab.Var1;
theta = tab.Var2;
omega = tab.Var3;

figure;
hold on; grid on
plot(t,omega);