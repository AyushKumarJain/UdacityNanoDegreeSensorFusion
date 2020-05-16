% Implement 1D CFAR using lagging cells on the given noise and target scenario.

% Close and delete all currently open figures
close all;

% Data_points
Ns = 1000;  %First we pick the number of samples for which we want to run the CFAR

% Generate random noise
s=randn(Ns,1); %then we generate random noise for the same amount of samples and take the absolute value of it

%We assign random amplitudes [8 9 4 11] to the noise samples at bin 100, 200, 350 and 700 to make them appear as mock Targets 
%Targets location. Assigning bin 100, 200, 300 and 700 as Targets with the amplitudes of 8, 9, 4, 11.
s([100 ,200, 300, 700])=[8 9 4 11];

%plot the output
plot(s);


% TODO: Apply CFAR to detect the targets by filtering the noise.

% 1. First we define the following:
% 1a. Training Cells
T=12;
% 1b. Guard Cells 
G=4;

% Offset : Adding room above noise threshold for desired SNR 
% Here we are working on linear values, hence we multiply the offset to the
% threshold value.
offset=5;

% Vector to hold threshold values 
threshold_cfar = [];

%Vector to hold final signal after thresholding
signal_cfar = [];

% 2. Slide window across the signal length
for i = 1:(Ns-(G+T+1))     
    
    %now for each step add the noise within all the training cells.
    noise_level = sum(s(i:i+T-1));

    % To determine the threshold take the average of summed noise and
    % multiply it with the offset.
    % We considered lagging noise only.
    threshold = (noise_level/T)*offset;
    threshold_cfar = [threshold_cfar, {threshold}];
    
    %Now pick the cell under the test which T+G cells away from the first
    %training cell and measure the signal level
    % 6. Measuring the signal within the CUT
    signal = s(i+T+G);

    %If the signal level at Cell Under Test is below threshold assign it to
    %Zero value
    % 8. Filter the signal above the threshold
    
    if(signal<threshold)
        signal=0;
    end

    signal_cfar = [signal_cfar, {signal}];
end


%% Here we plot the threshold, noise signal and thresholded signal

% plot the filtered signal
plot (cell2mat(signal_cfar),'g--');

% plot original sig, threshold and filtered signal within the same figure.
figure,plot(s);
hold on,plot(cell2mat(circshift(threshold_cfar,G)),'r--','LineWidth',2)
hold on, plot (cell2mat(circshift(signal_cfar,(T+G))),'g--','LineWidth',4);
legend('Signal','CFAR Threshold','detection')