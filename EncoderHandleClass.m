classdef EncoderHandleClass < handle

    properties

        ComPort = 'Nan';
        
        Hrealterm;                                               
        FilePointer=0;                                                   
        
        BufferSize=1;
        
    end

    methods
        function thisEncoder = EncoderHandleClass(comPort)          
            thisEncoder.ComPort = comPort;           
        end 

        function data = getdata(thisEncoder)
            data = capturedata(thisEncoder);
        end

        function serialData = capturedata(thisEncoder)
            % Reads data from the serial buffer, frames and parses these data.        
            
            [serialData, ~] = readdatabuffer(thisEncoder, inf);  % Read all available serial data from the com port
            
        end 

        function isInitialised = initialiserealterm(thisEncoder)
            % Initialises Realterm buffer.
            thisEncoder.Hrealterm = actxserver('realterm.realtermintf');   % Start Realterm as a server
            thisEncoder.Hrealterm.baud = 57600;
            thisEncoder.Hrealterm.TimerPeriod = 10000;                     % Set time-out to 10 seconds       
            thisEncoder.Hrealterm.Port = thisEncoder.ComPort;              % Assign the Shimmer Com Port number to the realterm server
            thisEncoder.Hrealterm.caption = strcat('Matlab Shimmer Realterm Server COM',thisEncoder.ComPort);   % Assign a title to the realterm server window
            thisEncoder.Hrealterm.windowstate = 1;                         % Minimise realterm server window
            realtermBufferDirectory = strcat(pwd,'\realtermBuffer');       % Define directory for realtermBuffer
            
            if ~(exist(realtermBufferDirectory,'dir'))                     % if realtermBuffer directory does not exist then create it
                mkdir(realtermBufferDirectory);
            end
            
            thisEncoder.Hrealterm.CaptureFile=strcat(realtermBufferDirectory,'\matlab_data_COM',thisEncoder.ComPort,'.dat');    % define realterm buffer file name
            disp(thisEncoder.Hrealterm.CaptureFile)
            isInitialised = true;
        end 

        function isOpen = opencomport(thisEncoder)
            % Open COM Port.
            initialiserealterm(thisEncoder);                               % Define and open realterm server
            try
                thisEncoder.Hrealterm.PortOpen = true;                     % Open the COM Port
            catch
                fprintf(strcat('Warning: opencomport - Unable to open Com Port ',thisEncoder.ComPort,'.\n'));
            end
            
            if(thisEncoder.Hrealterm.PortOpen~=0)                          % TRUE if the COM Port opened OK
                invoke(thisEncoder.Hrealterm,'startcapture');              % Enable realtime buffer
                thisEncoder.FilePointer = 0;                               % Set FilePointer to start of file
                isOpen=1;
            else
                disconnect(thisEncoder);                                   % TRUE if COM Port didnt open close realterm server
                isOpen=0;
            end
        end % opencomport

        function isCleared = clearreaddatabuffer(thisEncoder)
            % The buffer isnt really cleared, all available data is read
            [~, isCleared] = readdatabuffer(thisEncoder, inf);   % so that file pointer is set to end of file
            
        end 
                
        function isOpen = closecomport(thisEncoder)
            % Close COM Port.
            thisEncoder.Hrealterm.PortOpen=0;                            % Close the COM Port
            
            if(thisEncoder.Hrealterm.PortOpen~=0)                        % TRUE if the COM Port is still open
                isOpen=1;
                fprintf(strcat('Warning: closecomport - Unable to close COM',thisEncoder.ComPort,'.\n'));
            else
                isOpen=0;
                closerealterm(thisEncoder);
            end
        end % function closecomport
                
        function isClosed = closerealterm(thisEncoder)
            % Close the Realterm server.
            invoke(thisEncoder.Hrealterm,'stopcapture');
            isClosed = true;
            try                                                           % Try to close realterm server
                invoke(thisEncoder.Hrealterm,'close'); delete(thisEncoder.Hrealterm);
            catch
                isClosed = false;
                fprintf(strcat('Warning: closerealterm - Unable to close realterm for COM',thisEncoder.ComPort,'.'))
            end
            
        end 
        
        function [bufferedData, didFileOpen] = readdatabuffer(thisEncoder, nValues)
            % Reads data from the Realterm data buffer.
            bufferedData=[];
            
            fileId = fopen(thisEncoder.Hrealterm.CaptureFile, 'r');        % Open file with read only permission
            if (fileId ~= -1)                                              % TRUE if file was opened successfully
                didFileOpen = true;                                        % Set isFileOpen to 1 to indicate that the file was opened
                fseek(fileId,thisEncoder.FilePointer,'bof');               % Set file pointer to value stored from previous fread
                bufferedData=fread(fileId, nValues, '*char');             % Read data from the realterm data buffer
                thisEncoder.FilePointer = thisEncoder.FilePointer + length(bufferedData);  % Update FilePointer value to position of last value read
                fclose(fileId);
            else
                didFileOpen = false;                                             % Set isFileOpen to 0 to indicate that the file failed to open
                fprintf(strcat('Warning: readdatabuffer - Cannot open realterm capture file for COM',thisEncoder.ComPort,'.\n'));
            end
            
        end 
    end
end