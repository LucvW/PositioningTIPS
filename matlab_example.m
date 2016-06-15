%% Mathworks Matlab example code
% source: http://nl.mathworks.com/help/signal/examples/signal-generation-and-visualization.html?prodcode=ML


% Generate a linear chirp:
t = 0:0.001:2;                 % 2 secs @ 1kHz sample rate
ylin = chirp(t,0,1,150);       % Start @ DC, cross 150Hz at t=1sec

% Generate a quadratic chirp:
t = -2:0.001:2;                % +/-2 secs @ 1kHz sample rate
yq = chirp(t,100,1,200,'q');   % Start @ 100Hz, cross 200Hz at t=1sec

% Compute and display the spectrograms
subplot(211)
spectrogram(ylin,256,250,256,1E3,'yaxis');
title('Linear Chirp')
subplot(212)
spectrogram(yq,128,120,128,1E3,'yaxis');
title('Quadratic Chirp')
hgcf = gcf;
hgcf.Color = [1,1,1];