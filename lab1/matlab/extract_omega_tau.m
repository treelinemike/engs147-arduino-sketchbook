% restart
close all; clear; clc;
Ts = 0.005;  % [sec] sampling period

% load data
tab = readtable('pwm_n400.csv','Delimiter',',')

% compute time
time = tab.Var1;
time = time - time(1);
time = time /   1e6;

% compute speed in rad/s
omega = tab.Var3*(1/1440)*(2*pi);

[b,a] = butter(2,0.05,'low');
omega_filt = filtfilt(b,a,omega);

figure;
hold on; grid on;
plot(time,omega,'b-');
plot(time,omega_filt,'r-');
xlabel('\bfTime [s]');
ylabel('\bfSpeed [rad/s]');


omega_ss = omega_filt(end)  % [rad/s]
if(sign(omega_ss) > 0)
    tau = time(find(omega_filt >= 0.632*omega_ss,1,'first'))
else
    tau = time(find(omega_filt <= 0.632*omega_ss,1,'first'))
end