%%Calculate the range in meters of four targets with respective measured beat frequencies [0 MHz, 1.1 MHz, 13 MHz, 24 MHz].
% Given 
% The radar maximum range = 300m
% The range resolution = 1m
% The speed of light c = 3*10^8

% TODO: Find the Bsweep of chirp for 1m resolution 
c = 3*10^8;           %speed of light in meters/sec
delta_r = 1;          %range resolution in meters
Bsweep = c/2*delta_r; %Bsweep calculation


% TODO: Calculate the chirp time based on the Radar's Max Range
range_max = 300;            %given radar's max range
Ts = 5.5*(range_max*2/c);   %5.5 times of the trip time for maximum range

%TODO: define the frequency shifts
beat_freq = [0 1.1e6 13e6 24e6];              %given beat frequencies for all four targets
calculated_range = c*Ts*beat_freq/(2*Bsweep); %range calculation

%display the calculate range
disp(calculated_range);
