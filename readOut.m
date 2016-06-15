%% TIPS position-system
% Readout arduino
% Uses functions setupSerial and readTemp provided by http://www.instructables.com/id/Arduino-and-Matlab-let-them-talk-using-serial-comm/step3/Matlab-lets-tame-the-beast/
% C. Treffers & L. van Wietmarschen, TU Delft 23-5-2016
%% start program
clc;
clear all;
close all;
comPort = 'com5'; % select com port of arduino connection
pos(3,10) = 0; %make position output array 
nor(3,10) = 0; %make normal output array
%% creating serial communication
if(~exist('serialFlag','var'))
    [arduino,serialFlag] = setupSerial(comPort);
end
%% wait for start command
status = 0;
start = 'startStream';  %start signal of data stream
while (status == 0)     %waiting loop for start signal
    out = readTempString(arduino, 'R'); % read data from arduino
    if (strcmp(out,start)) % on start signal exit waiting loop
        status = 1
    end
end

%% readout data
n = 1;
while(1)
    [pos(:,n), nor(:,n), error] = getArduinoData(arduino);
    if error > 0 
        break;
    end
    n = n + 1;
end
switch error
    case 1
        disp('error: Data skew detected in last value');
        disp(['error occured at measurement ', num2str(n-1)]);
    case 2
        disp(['Data stream was stopped at measurement ',num2str(n-1)]);
    case 3
        disp('error: Unknown error occured');
        disp(['error occured at measurement ', num2str(n-1)]);
end
fclose(arduino);