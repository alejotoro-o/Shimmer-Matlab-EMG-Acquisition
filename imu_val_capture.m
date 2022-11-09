close all
clear all
clc

% Definicion parametros de captura

comPort1 = '3'; % D87E
comPort2 = '7'; % D9C8
comport3 = '10'; % Encoder

% S = serialport(comport3, 76800, 'timeout', 10);
% flush(S)

% Seleccion algoritmo de estimacion de orientacion
beta = 0.6;
Kp = 10;
Ki = 1;

%addpath('./Resources/')                                                    % directory containing supporting functions

% Definicion despositivos Shimmer
shimmer1 = ShimmerHandleClass(comPort1); %EMG + IMU - Shimmer D87E
shimmer2 = ShimmerHandleClass(comPort2); %EMG + IMU - Shimmer D9C8
encoder = EncoderHandleClass(comport3);

SensorMacros = SetEnabledSensorsMacrosClass;                               % assign user friendly macros for setenabledsensors

fs = 1024; % Frecuencia de muestreo                                                                 % sample rate in [Hz]
firsttime = true;
AHRSinit = true;

NO_SAMPLES_IN_PLOT = 5000; % Numero de muestras en la figura                                                         % Delay (in seconds) between data read operations
DELAY_PERIOD = 0; % Delay entre iteraciones del ciclo
numSamples = 0;

% Coneccion con dispositivos Shimmer
if (shimmer1.connect && shimmer2.connect)

    encoder.opencomport;

    % Configuracion Shimmer 1
    shimmer1.setsamplingrate(fs); % Select sampling rate
    shimmer1.setinternalboard('EXG'); % Select internal expansion board; select 'EMG' to enable SENSOR_EXG1
    shimmer1.disableallsensors; % Disable other sensors
    shimmer1.setenabledsensors(SensorMacros.EMG,0,SensorMacros.ACCEL,1,SensorMacros.GYRO,1,SensorMacros.MAG,1);                         % Enable SENSOR_EXG1, disable other sensors
    shimmer1.setgyroinusecalibration(1);

    % Configuracion Shimmer 2
    shimmer2.setsamplingrate(fs); % Select sampling rate
    shimmer2.setinternalboard('EXG'); % Select internal expansion board; select 'EMG' to enable SENSOR_EXG1
    shimmer2.disableallsensors; % Disable other sensors
    shimmer2.setenabledsensors(SensorMacros.EMG,0,SensorMacros.ACCEL,1,SensorMacros.GYRO,1,SensorMacros.MAG,1); 
    shimmer2.setgyroinusecalibration(1);


    if (shimmer1.start && shimmer2.start)                                                  % TRUE if the shimmer starts streaming 
                                              
        plotDataTheta = [];

        h.figure1 = figure('Name','IMU');                    % Create a handle to figure for plotting data from shimmer
        set(h.figure1, 'Position', [600, 150, 800, 600]);

        % Controles interfaz grafica
        stopHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'Salir', ...
                         'Callback', 'delete(gcbf)',...
                         'position',[520 5 80 20]);

        recordHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'Grabar', ...
                         'Callback', @startRecFn,...
                         'position',[380 5 80 20]);
        recordHandle.UserData = false;

        fileHandle = uicontrol('Style', 'edit', ...
                         'position',[240 5 80 20]);

        cal1Handle = uicontrol('Style', 'PushButton', ...
                         'String', 'Cal 1', ...
                         'Callback', 'cal1Handle.UserData = true;',...
                         'position',[620 5 80 20]);
        cal1Handle.UserData = false;

        lambda_mad = [];
        lambda_ekf = [];
        lambda_mah = [];

        while 1      
            
            pause(DELAY_PERIOD); % Pause for this period of time on each iteration to allow data to arrive in the buffer
            
            [newData,signalNameArray,signalFormatArray,signalUnitArray] = shimmer1.getdata('c'); % Read the latest data from shimmer data buffer, signalFormatArray defines the format of the data and signalUnitArray the unit
            [newData2,signalNameArray2,signalFormatArray2,signalUnitArray2] = shimmer2.getdata('c');         
            edata = encoder.getdata();
            

            if ~isempty(newData) && ~isempty(newData2) && ~isempty(edata)                                      % TRUE if new data has arrived
              
                encoderData = split(edata','_');
                encoderData = str2double(encoderData(2:end-1));

                % Igualar cantidad de datos en los buffers
                dataSize = [size(newData,1), size(newData2,1), size(encoderData,1)];
                minSize = min(dataSize);
    
                newData = newData(end-minSize+1:end,:);
                newData2 = newData2(end-minSize+1:end,:);
                encoderData = encoderData(end-minSize+1:end);

                % Indices Shimmer 1
                chIndex1(1) = find(ismember(signalNameArray, 'Low Noise Accelerometer X'));
                chIndex1(2) = find(ismember(signalNameArray, 'Low Noise Accelerometer Y'));
                chIndex1(3) = find(ismember(signalNameArray, 'Low Noise Accelerometer Z'));
                chIndex1(4) = find(ismember(signalNameArray, 'Gyroscope X'));
                chIndex1(5) = find(ismember(signalNameArray, 'Gyroscope Y'));
                chIndex1(6) = find(ismember(signalNameArray, 'Gyroscope Z'));
                chIndex1(7) = find(ismember(signalNameArray, 'Magnetometer X'));
                chIndex1(8) = find(ismember(signalNameArray, 'Magnetometer Y'));
                chIndex1(9) = find(ismember(signalNameArray, 'Magnetometer Z'));

                accelData1 = newData(:,[chIndex1(1), chIndex1(2), chIndex1(3)]);
                gyroData1 = deg2rad(newData(:,[chIndex1(4), chIndex1(5), chIndex1(6)]));
                magData1 = (newData(:,[chIndex1(7), chIndex1(8), chIndex1(9)])/10000)*1000000;

                % Indices Shimmer 2
                chIndex2(1) = find(ismember(signalNameArray2, 'Low Noise Accelerometer X'));
                chIndex2(2) = find(ismember(signalNameArray2, 'Low Noise Accelerometer Y'));
                chIndex2(3) = find(ismember(signalNameArray2, 'Low Noise Accelerometer Z'));
                chIndex2(4) = find(ismember(signalNameArray2, 'Gyroscope X'));
                chIndex2(5) = find(ismember(signalNameArray2, 'Gyroscope Y'));
                chIndex2(6) = find(ismember(signalNameArray2, 'Gyroscope Z'));
                chIndex2(7) = find(ismember(signalNameArray2, 'Magnetometer X'));
                chIndex2(8) = find(ismember(signalNameArray2, 'Magnetometer Y'));
                chIndex2(9) = find(ismember(signalNameArray2, 'Magnetometer Z'));

                accelData2 = newData2(:,[chIndex2(1), chIndex2(2), chIndex2(3)]);
                gyroData2 = deg2rad(newData2(:,[chIndex2(4), chIndex2(5), chIndex2(6)]));
                magData2 = (newData2(:,[chIndex2(7), chIndex2(8), chIndex2(9)])/10000)*1000000;

                % Inicializacion estimacion orientacion
                if AHRSinit

                    AHRSinit = false;

                    qInit1 = compact(ecompass(accelData1(1,:),magData1(1,:)));
                    qInit2 = compact(ecompass(accelData2(1,:),magData2(1,:)));

                    AHRS_mad1 = MadgwickAHRS('SamplePeriod', 1/fs, 'Beta', beta, 'Quaternion', qInit1);
                    AHRS_mad2 = MadgwickAHRS('SamplePeriod', 1/fs, 'Beta', beta, 'Quaternion', qInit2);

                    AHRS_ekf1 = EKF('SampleRate', fs, 'Quaternion', qInit1);
                    AHRS_ekf2 = EKF('SampleRate', fs, 'Quaternion', qInit2);

                    AHRS_mah1 = MahonyAHRS('SamplePeriod', 1/fs, 'Kp', Kp, 'Ki', Ki, 'Quaternion', qInit2);
                    AHRS_mah2 = MahonyAHRS('SamplePeriod', 1/fs, 'Kp', Kp, 'Ki', Ki, 'Quaternion', qInit2);
                end             

                % Estimar orientacion IMUs
                qmad1 = [];
                qmad2 = [];
                qekf1 = [];
                qekf2 = [];
                qmah1 = [];
                qmah2 = [];
                for t = 1:minSize
                    AHRS_mad1.Update(accelData1(t,:),gyroData1(t,:),magData1(t,:));
                    qmad1 = [qmad1; AHRS_mad1.Quaternion];
                    AHRS_ekf1.Update(accelData1(t,:),gyroData1(t,:),magData1(t,:));
                    qekf1 = [qekf1; AHRS_ekf1.Quaternion];
                    AHRS_mah1.Update(accelData1(t,:),gyroData1(t,:),magData1(t,:));
                    qmah1 = [qmah1; AHRS_mah1.Quaternion];

                    AHRS_mad2.Update(accelData2(t,:),gyroData2(t,:),magData2(t,:));
                    qmad2 = [qmad2; AHRS_mad2.Quaternion];
                    AHRS_ekf2.Update(accelData2(t,:),gyroData2(t,:),magData2(t,:));
                    qekf2 = [qekf2; AHRS_ekf2.Quaternion];
                    AHRS_mah2.Update(accelData2(t,:),gyroData2(t,:),magData2(t,:));
                    qmah2 = [qmah2; AHRS_mah2.Quaternion];
                end

                % Calcular angulos
                theta_mad = getAngle(qmad1,qmad2,lambda_mad);
                theta_ekf = getAngle(qekf1,qekf2,lambda_ekf);
                theta_mah = getAngle(qmah1,qmah2,lambda_mah);

                plotDataTheta = [plotDataTheta; [theta_mad, theta_ekf, theta_mah, encoderData]];

                % Numero de muestras en la grafica
                numPlotSamples = size(plotDataTheta,1);

                % Numero de muestras totales
                numSamples = numSamples + size(newData,1);
                               
                if numSamples > NO_SAMPLES_IN_PLOT
                    plotDataTheta = plotDataTheta(numPlotSamples-NO_SAMPLES_IN_PLOT+1:end,:);
                end
                sampleNumber = max(numSamples-NO_SAMPLES_IN_PLOT+1,1):numSamples;

                % Grafica angulo flexo - extension                                      
                plot(sampleNumber,plotDataTheta(:,1),sampleNumber,plotDataTheta(:,2),sampleNumber,plotDataTheta(:,3),sampleNumber,plotDataTheta(:,4));
                xlim([sampleNumber(1) sampleNumber(end)]);
                legend('Madgwick','EKF','Mahony','Encoder')

            else

                disp("NO DATA")

            end        

            if ~ishandle(stopHandle)
                disp('Captura detenida');
                break;
            end

            if recordHandle.UserData
                if isempty(fileHandle.String)
                    disp('Introducir nombre de archivo')
                    recordHandle.UserData = false;
                else
                    fileName = "D:\Documentos\Datasets\IMU Validacion\" + fileHandle.String + ".txt";

                    data = [theta_mad, theta_ekf, theta_mah, encoderData];
                    dlmwrite(fileName, data, '-append', 'delimiter', '\t','precision',16);

                end
            end

            % Calibrar orientacion q1 y q2
            if cal1Handle.UserData
                disp("Calibracion Flexion - Extension");
                lambda_mad = quatmultiply(quatconjugate(qmad2(end,:)),qmad1(end,:));
                lambda_ekf = quatmultiply(quatconjugate(qekf2(end,:)),qekf1(end,:));
                lambda_mah = quatmultiply(quatconjugate(qmah2(end,:)),qmah1(end,:));
                cal1Handle.UserData = false;
            end
        end  
        
        shimmer1.stop;   
        shimmer2.stop; 

    end 

end

shimmer1.disconnect;
shimmer2.disconnect;
encoder.closecomport;

clear shimmer1;
clear shimmer2;

% Callback para boton de inicio de grabacion
function startRecFn(recordHandle, eventdata)

    recordHandle.UserData = ~recordHandle.UserData;
    
    if recordHandle.UserData
        disp("Grabacion iniciada")
    else
        disp("Grabacion finalizada")
    end

end
