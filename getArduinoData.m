% Readout arduino
% Uses functions setupSerial and readTemp provided by http://www.instructables.com/id/Arduino-and-Matlab-let-them-talk-using-serial-comm/step3/Matlab-lets-tame-the-beast/
% C. Treffers & L. van Wietmarschen, TU Delft 14-6-2016

function [position, normal, error] = getArduinoData(arduino)
% This function will attempt to collect the position and normal from the
% arduino. It will first check if the data stream is stopped, if not it
% will find the first expected element and start collecting data from
% there.
% errorcodes:   0 - no error occured
%               1 - data skew detected
%               2 - data stream was stopped
%               3 - unknown error occured

skewCheck(6) = 0;   % used to detect data skew
error = 3;          % starting error, unknown cause error
position(3) = 0;    % make position output array 
normal(3) = 0;      % make normal output array
xposDetect = 0;     % no xpos is given trough
while (1) % readout loop
    output = readTempString(arduino, 'R');  % read data from serial port
    outSplit = strsplit(output,':');        % split string on ':'
    
    if (strcmp(output,'stopStream'));       % if stream is stopped, stop all
        error = 2;      % stream stopped error
        break;
    end
    if (strcmp('xpos',outSplit{1}))
        xposDetect = 1; % find the start of the data stream
    end
    
    if (xposDetect)     % only read out data if starting at start
    switch outSplit{1}  % determine which value is received
        case 'xpos'
            position(1) = str2double(outSplit{2});
            skewCheck(1) = skewCheck(1) + 1;
        case 'ypos'
            position(2) = str2double(outSplit{2});
            skewCheck(2) = skewCheck(2) + 1;
        case 'zpos'
            position(3) = str2double(outSplit{2});
            skewCheck(3) = skewCheck(3) + 1;
        case 'xnor'
            normal(1) = str2double(outSplit{2});
            skewCheck(4) = skewCheck(4) + 1;
        case 'ynor'
            normal(2) = str2double(outSplit{2});
            skewCheck(5) = skewCheck(5) + 1;
        case 'znor'
            normal(3) = str2double(outSplit{2});
            skewCheck(6) = skewCheck(6) + 1;
    end
    end
    if sum(skewCheck > 1)% check for data that was read in more than once
        error = 1;       % data skew detected
        break;        
    elseif isequal(skewCheck,[1,1,1,1,1,1])
        error = 0;       % no errors occured, program succesfull
        break;
    end
end
end