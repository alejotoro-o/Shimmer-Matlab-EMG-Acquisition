close all
clear all
clc

% Definicion parametros de captura

comPort1 = '3'; % D87E
comPort2 = '7'; % D9C8

% Definicion despositivos Shimmer

shimmer1 = ShimmerHandleClass(comPort1); %EMG + IMU - Shimmer D87E
shimmer2 = ShimmerHandleClass(comPort2); %EMG + IMU - Shimmer D9C8
SensorMacros = SetEnabledSensorsMacrosClass;                               % assign user friendly macros for setenabledsensors

% shimmer1.enabletimestampunix(1);
% shimmer2.enabletimestampunix(1);

fs = 1024; % Frecuencia de muestreo                                                                 % sample rate in [Hz]
firsttime = true;
AHRSinit = true;

% Note: these constants are only relevant to this examplescript and are not used
% by the ShimmerHandle Class
NO_SAMPLES_IN_PLOT = 5000; % Numero de muestras en la figura                                                         % Delay (in seconds) between data read operations
DELAY_PERIOD = 0.2; % Delay entre iteraciones del ciclo
numSamples = 0;

% Configuracion de captura

% Opciones de filtrado
fm = 60; % mains frequency [Hz]
fchp = 10; % corner frequency highpassfilter [Hz]; Shimmer recommends 5Hz to remove DC-offset and movement artifacts
nPoles = 4; % number of poles (HPF, LPF)
pbRipple = 0.5; % pass band ripple (%)

HPF = true; % enable (true) or disable (false) highpass filter
LPF = true; % enable (true) or disable (false) lowpass filter
BSF = true; % enable (true) or disable (false) bandstop filter

% highpass filters for ExG channels
if (HPF)
    % Filtro Shimmer 1
    hpfexg1ch1 = FilterClass(FilterClass.HPF,fs,fchp,nPoles,pbRipple);
    hpfexg1ch2 = FilterClass(FilterClass.HPF,fs,fchp,nPoles,pbRipple);
    % Filtro Shimmer 2
    hpfexg2ch1 = FilterClass(FilterClass.HPF,fs,fchp,nPoles,pbRipple);
    hpfexg2ch2 = FilterClass(FilterClass.HPF,fs,fchp,nPoles,pbRipple);
end
% lowpass filters for ExG channels
if (LPF)
    % Filtro Shimmer 1
    lpfexg1ch1 = FilterClass(FilterClass.LPF,fs,fs/2-1,nPoles,pbRipple);
    lpfexg1ch2 = FilterClass(FilterClass.LPF,fs,fs/2-1,nPoles,pbRipple);
    % Filtro Shimmer 2
    lpfexg2ch1 = FilterClass(FilterClass.LPF,fs,fs/2-1,nPoles,pbRipple);
    lpfexg2ch2 = FilterClass(FilterClass.LPF,fs,fs/2-1,nPoles,pbRipple);
end
% bandstop filters for ExG channels;
% cornerfrequencies at +1Hz and -1Hz from mains frequency
if (BSF)
    % Filtro Shimmer 1
    bsfexg1ch1 = FilterClass(FilterClass.LPF,fs,[fm-1,fm+1],nPoles,pbRipple);
    bsfexg1ch2 = FilterClass(FilterClass.LPF,fs,[fm-1,fm+1],nPoles,pbRipple);
    % Filtro Shimmer 2
    bsfexg2ch1 = FilterClass(FilterClass.LPF,fs,[fm-1,fm+1],nPoles,pbRipple);
    bsfexg2ch2 = FilterClass(FilterClass.LPF,fs,[fm-1,fm+1],nPoles,pbRipple);
end


% Coneccion con dispositivos Shimmer
if (shimmer1.connect && shimmer2.connect)

    % Configuracion Shimmer 1
    shimmer1.setsamplingrate(fs); % Select sampling rate
    shimmer1.setinternalboard('EXG'); % Select internal expansion board; select 'EMG' to enable SENSOR_EXG1
    shimmer1.disableallsensors; % Disable other sensors
    shimmer1.setenabledsensors(SensorMacros.EMG,1,SensorMacros.ACCEL,1,SensorMacros.GYRO,1,SensorMacros.MAG,1);                         % Enable SENSOR_EXG1, disable other sensors
    shimmer1.setgyroinusecalibration(1);

    % Configuracion Shimmer 2
    shimmer2.setsamplingrate(fs); % Select sampling rate
    shimmer2.setinternalboard('EXG'); % Select internal expansion board; select 'EMG' to enable SENSOR_EXG1
    shimmer2.disableallsensors; % Disable other sensors
    shimmer2.setenabledsensors(SensorMacros.EMG,1,SensorMacros.ACCEL,1,SensorMacros.GYRO,1,SensorMacros.MAG,1); 
    shimmer2.setgyroinusecalibration(1);


    if (shimmer1.start && shimmer2.start)                                                  % TRUE if the shimmer starts streaming 
                                              
        plotDataEMG1 = [];
        plotDataEMG2 = [];
        plotDataTheta = [];

        h.figure1 = figure('Name','Shimmer EMG signals');                    % Create a handle to figure for plotting data from shimmer
        set(h.figure1, 'Position', [600, 150, 800, 600]);

        % Controles interfaz grafica
        stopHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'Salir', ...
                         'Callback', 'delete(gcbf)',...
                         'position',[420 5 80 20]);

        recordHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'Grabar', ...
                         'Callback', @startRecFn,...
                         'position',[320 5 80 20]);
        recordHandle.UserData = false;

        fileHandle = uicontrol('Style', 'edit', ...
                         'position',[220 5 80 20]);

        cal1Handle = uicontrol('Style', 'PushButton', ...
                         'String', 'Cal 1', ...
                         'Callback', 'cal1Handle.UserData = true;',...
                         'position',[520 5 80 20]);
        cal1Handle.UserData = false;

        lambda1 = [];                                                       

        while 1      

            pause(DELAY_PERIOD); % Pause for this period of time on each iteration to allow data to arrive in the buffer

            [newData,signalNameArray,signalFormatArray,signalUnitArray] = shimmer1.getdata('c'); % Read the latest data from shimmer data buffer, signalFormatArray defines the format of the data and signalUnitArray the unit
            [newData2,signalNameArray2,signalFormatArray2,signalUnitArray2] = shimmer2.getdata('c');

            if ~isempty(newData) && ~isempty(newData2)                                           % TRUE if new data has arrived
                
                % Igualar cantidad de datos en los buffers
                dataSize = [size(newData,1), size(newData2,1)];
                minSize = min(dataSize);
    
                newData = newData(end-minSize+1:end,:);
                newData2 = newData2(end-minSize+1:end,:);

                % Indices Shimmer 1
                chIndex1(1) = find(ismember(signalNameArray, 'EMG CH1'));
                chIndex1(2) = find(ismember(signalNameArray, 'EMG CH2'));

                chIndex1(3) = find(ismember(signalNameArray, 'Low Noise Accelerometer X'));
                chIndex1(4) = find(ismember(signalNameArray, 'Low Noise Accelerometer Y'));
                chIndex1(5) = find(ismember(signalNameArray, 'Low Noise Accelerometer Z'));
                chIndex1(6) = find(ismember(signalNameArray, 'Gyroscope X'));
                chIndex1(7) = find(ismember(signalNameArray, 'Gyroscope Y'));
                chIndex1(8) = find(ismember(signalNameArray, 'Gyroscope Z'));
                chIndex1(9) = find(ismember(signalNameArray, 'Magnetometer X'));
                chIndex1(10) = find(ismember(signalNameArray, 'Magnetometer Y'));
                chIndex1(11) = find(ismember(signalNameArray, 'Magnetometer Z'));

                EMGData1 = newData(:,[chIndex1(1), chIndex1(2)]);
                EMGDataFiltered1 = EMGData1;

                accelData1 = newData(:,[chIndex1(3), chIndex1(4), chIndex1(5)]);
                gyroData1 = deg2rad(newData(:,[chIndex1(6), chIndex1(7), chIndex1(8)]));
                magData1 = (newData(:,[chIndex1(9), chIndex1(10), chIndex1(11)])/10000)*1000000;

                % Indices Shimmer 2
                chIndex2(1) = find(ismember(signalNameArray2, 'EMG CH1'));
                chIndex2(2) = find(ismember(signalNameArray2, 'EMG CH2'));

                chIndex2(3) = find(ismember(signalNameArray2, 'Low Noise Accelerometer X'));
                chIndex2(4) = find(ismember(signalNameArray2, 'Low Noise Accelerometer Y'));
                chIndex2(5) = find(ismember(signalNameArray2, 'Low Noise Accelerometer Z'));
                chIndex2(6) = find(ismember(signalNameArray2, 'Gyroscope X'));
                chIndex2(7) = find(ismember(signalNameArray2, 'Gyroscope Y'));
                chIndex2(8) = find(ismember(signalNameArray2, 'Gyroscope Z'));
                chIndex2(9) = find(ismember(signalNameArray2, 'Magnetometer X'));
                chIndex2(10) = find(ismember(signalNameArray2, 'Magnetometer Y'));
                chIndex2(11) = find(ismember(signalNameArray2, 'Magnetometer Z'));

                EMGData2 = newData2(:,[chIndex2(1), chIndex2(2)]);
                EMGDataFiltered2 = EMGData2;

                accelData2 = newData2(:,[chIndex2(3), chIndex2(4), chIndex2(5)]);
                gyroData2 = deg2rad(newData2(:,[chIndex2(6), chIndex2(7), chIndex2(8)]));
                magData2 = (newData2(:,[chIndex2(9), chIndex2(10), chIndex2(11)])/10000)*1000000;

                % Inicializacion estimacion orientacion
                if AHRSinit

                    AHRSinit = false;

                    % Shimmer 1
                    qInit1 = compact(ecompass(accelData1(1,:),magData1(1,:)));
                    AHRS1 = MadgwickAHRS('SamplePeriod', 1/fs, 'Beta', 0.6, 'Quaternion', qInit1);

                    % Shimmer 2
                    qInit2 = compact(ecompass(accelData2(1,:),magData2(1,:)));
                    AHRS2 = MadgwickAHRS('SamplePeriod', 1/fs, 'Beta', 0.6, 'Quaternion', qInit2);

                end

                % Filtrar canales EMG
                if HPF % filter newData with highpassfilter to remove DC-offset
                    EMGDataFiltered1(:,1) = hpfexg1ch1.filterData(EMGDataFiltered1(:,1));
                    EMGDataFiltered1(:,2) = hpfexg1ch2.filterData(EMGDataFiltered1(:,2));

                    EMGDataFiltered2(:,1) = hpfexg2ch1.filterData(EMGDataFiltered2(:,1));
                    EMGDataFiltered2(:,2) = hpfexg2ch2.filterData(EMGDataFiltered2(:,2));
                end
                
                if BSF % filter highpassfiltered data with bandstopfilter to suppress mains interference
                    EMGDataFiltered1(:,1) = bsfexg1ch1.filterData(EMGDataFiltered1(:,1));
                    EMGDataFiltered1(:,2) = bsfexg1ch2.filterData(EMGDataFiltered1(:,2));

                    EMGDataFiltered2(:,1) = bsfexg2ch1.filterData(EMGDataFiltered2(:,1));
                    EMGDataFiltered2(:,2) = bsfexg2ch2.filterData(EMGDataFiltered2(:,2));
                end
                
                if LPF % filter bandstopfiltered data with lowpassfilter to avoid aliasing
                    EMGDataFiltered1(:,1) = lpfexg1ch1.filterData(EMGDataFiltered1(:,1));
                    EMGDataFiltered1(:,2) = lpfexg1ch2.filterData(EMGDataFiltered1(:,2));

                    EMGDataFiltered2(:,1) = lpfexg2ch1.filterData(EMGDataFiltered2(:,1));
                    EMGDataFiltered2(:,2) = lpfexg2ch2.filterData(EMGDataFiltered2(:,2));
                end

                % Estimar orientacion IMUs
                q1 = [];
                q2 = [];
                for t = 1:minSize
                    AHRS1.Update(accelData1(t,:),gyroData1(t,:),magData1(t,:));
                    q1 = [q1; AHRS1.Quaternion];

                    AHRS2.Update(accelData2(t,:),gyroData2(t,:),magData2(t,:));
                    q2 = [q2; AHRS2.Quaternion];
                end

                % Calcular angulos
                theta1 = getAngle(q1,q2,lambda1);

                % Definir datos para graficar
                plotDataEMG1 = [plotDataEMG1; EMGDataFiltered1];                            % Update the plotData buffer with the new ECG data
                plotDataEMG2 = [plotDataEMG2; EMGDataFiltered2];

                plotDataTheta = [plotDataTheta; theta1];

                % Numero de muestras en la grafica
                numPlotSamples = size(plotDataEMG1,1);

                % Numero de muestras totales
                numSamples = numSamples + size(newData,1);
                               
                if numSamples > NO_SAMPLES_IN_PLOT
                    plotDataEMG1 = plotDataEMG1(numPlotSamples-NO_SAMPLES_IN_PLOT+1:end,:);
                    plotDataEMG2 = plotDataEMG2(numPlotSamples-NO_SAMPLES_IN_PLOT+1:end,:);
                    plotDataTheta = plotDataTheta(numPlotSamples-NO_SAMPLES_IN_PLOT+1:end,:);
                end
                sampleNumber = max(numSamples-NO_SAMPLES_IN_PLOT+1,1):numSamples;

                % Grafica angulo flexo - extension
                subplot(3,2,[1 2]);                                       
                plot(sampleNumber,plotDataTheta);                                                    
                xlim([sampleNumber(1) sampleNumber(end)]);
                title('Elbow Angle')
                ylabel('Angle [deg]')
                xlabel('Samples')
                
                % Grafica EMG biceps
                subplot(3,2,3);                                      
                plot(sampleNumber,plotDataEMG1(:,1));                                                 
                xlim([sampleNumber(1) sampleNumber(end)]);
                title('EMG Biceps')
                ylabel('Amplitud')

                % Grafica EMG triceps
                subplot(3,2,5);                                   
                plot(sampleNumber,plotDataEMG1(:,2));                                        
                xlim([sampleNumber(1) sampleNumber(end)]);
                title('EMG Triceps')
                ylabel('Amplitud')
                xlabel('Samples')

                % Grafica EMG supinacion
                subplot(3,2,4);                                      
                plot(sampleNumber,plotDataEMG2(:,1));                                     
                xlim([sampleNumber(1) sampleNumber(end)]);
                title('EMG Branchioradialis')

                % Grafica EMG pronacion
                subplot(3,2,6);                                      
                plot(sampleNumber,plotDataEMG2(:,2));                                       
                xlim([sampleNumber(1) sampleNumber(end)]);
                title('EMG Pronator')
                xlabel('Samples')
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
                    fileName = "D:\Documentos\Datasets\Grabaciones 3\" + fileHandle.String + ".txt";

                    %Datos: [shimmer 1 ch1, shimmer 1 ch2, shimmer 2 ch1, shimmer 2 ch2, theta 1]
                    data = [EMGDataFiltered1, EMGDataFiltered2, theta1];
                    dlmwrite(fileName, data, '-append', 'delimiter', '\t','precision',16);

                end
            end

            % Calibrar orientacion q1 y q2
            if cal1Handle.UserData
                disp("Calibracion Flexion - Extension");
                lambda1 = quatmultiply(quatconjugate(q2(end,:)),q1(end,:));
                cal1Handle.UserData = false;
            end
        end  
        
        shimmer1.stop;   
        shimmer2.stop; 
    end 

end

shimmer1.disconnect;
shimmer2.disconnect;

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
