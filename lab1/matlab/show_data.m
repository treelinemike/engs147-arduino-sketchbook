% restart
close all; clear; clc;

figure;
hold on; grid on;

tab = readtable('test_250us.csv');
time = tab.Var1;
time = time - time(1);
time = time /   1e6;

plot(time,tab.Var3,'r-');


tab = readtable('test_10ms.csv');
time = tab.Var1;
time = time - time(1);
time = time / 1e6;
plot(time,tab.Var3,'b-','LineWidth',1.6);

tab = readtable('test_50ms.csv');
time = tab.Var1;
time = time - time(1);
time = time / 1e6;
plot(time,tab.Var3,'g-','LineWidth',1.6);

tab = readtable('test_250ms.csv');
time = tab.Var1;
time = time - time(1);
time = time / 1e6;
plot(time,tab.Var3,'c-','LineWidth',1.6);

legend('250\mus','10ms','50ms','250ms','Location','NorthWest')
xlabel('\bfTime [s]');
ylabel('\bfEncoder Speed [count/s]');


figure;
tab = readtable('test.csv');
time = tab.Var1;
time = time - time(1);
time = time / 1e6;
plot(time,tab.Var3,'b-','LineWidth',1.6);

% figure
%plot(diff(time))
%figure
% plot(time,[0;diff(tab.Var2)./diff(tab.Var1/1e6)],'r--');