clear all
close all
clc

filePath = "1_sup_0_5_74.txt";

data = dlmread("D:\Documentos\Datasets\Grabaciones 3\" + filePath);

dSize = size(data,1);

theta = data(:,5);
emg1 = data(:,1);
emg2 = data(:,2);
emg3 = data(:,3);
emg4 = data(:,4);

subplot(5,1,1)
plot(theta)
ylabel('Grados')

subplot(5,1,2)
plot(emg1)
ylabel('Amp')

subplot(5,1,3)
plot(emg2)
ylabel('Amp')

subplot(5,1,4)
plot(emg3)
ylabel('Amp')

subplot(5,1,5)
plot(emg4)
ylabel('Amp')
xlabel('Muestras')

