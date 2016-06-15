%% Test of the equation of the yaw of the orientation determination
% Charlotte Treffers, TU Delft, June 2016
%%
pitch = 0;              % pitch of 0 degrees
roll = 90 * pi/180;     % pitch of 90 degrees
mx = 0;
my = 0;                 % magnetic field measured by the z magnetometer
mz = -1;                % in the -z direction
magx = mz*sin(roll) - my*cos(roll);
magy = mx*cos(pitch) + my*sin(pitch)*sin(roll) + mz*sin(pitch)*cos(roll);
yaw = atan2(magx,magy) * 180/pi %calculate yaw