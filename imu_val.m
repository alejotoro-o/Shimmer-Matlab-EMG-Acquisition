clear all
close all
clc

fileName = 'final4';

data = dlmread('D:\Documentos\Datasets\IMU Validacion\' + string(fileName) + '.txt');

m = size(data,1);
lim = 33144;

mae_mad = sum(abs(data(1:lim,1) - data(1:lim,4)))/m;
mae_ekf = sum(abs(data(1:lim,2) - data(1:lim,4)))/m;
mae_mah = sum(abs(data(1:lim,3) - data(1:lim,4)))/m;
disp(mae_mad)
disp(mae_ekf)
disp(mae_mah)

samples = 1:lim;

figure(1)
plot(samples, data(1:lim,1),samples, data(1:lim,2),samples, data(1:lim,3),samples, data(1:lim,4))
xlabel('Samples')
ylabel('Angle [deg]')
legend('Madgwick','EKF','Mahony','Encoder')