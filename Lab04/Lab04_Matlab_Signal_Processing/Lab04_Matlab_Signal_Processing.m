clc, clearvars, close all

%% Lab04: Generation, Processing, and Analysis of Sine Waves

%% 1) Read Data
% Load the provided signal 'signal.csv' using the 'csvread()' function and
% split the signal and time into two separate vectors.

fprintf('1) Start.\n')

data = readmatrix('signal.csv');
time = data(:, 4);
signal = data(:, 5);

% Plot the signal
figure(1)
subplot(2,1,1);
plot(time, signal);
title('Original Signal');
xlabel('Time (s)');
ylabel('Amplitude (V)');
grid on;

fprintf('1) Done.\n')

%% 2/3) Butterworth Filter
% Design a Butterworth filter that fulfills the required characteristics 
% given in the assignment description. Use the built-in functions of
% Matlab. The 'doc' and 'help' functions might be useful to obtain detailed
% information.

% 2) First, calculate the required filter order and cutoff frequency and
% print the result.

fprintf('2) Start.\n')

% Calculate Sampling Frequency (Fs) from time vector
dt =  time(2) - time(1);
Fs = 1 / dt;

fprintf('Sampling Frequency: %.2f Hz\n', Fs);

% Define Filter Requirements
Wp_Hz = 30;
Ws_Hz = 200;
Rp = 3;
Rs = 40;

% Normalize frequencies (Nyquist = Fs/2)
Wp = Wp_Hz / (Fs/2);
Ws = Ws_Hz / (Fs/2);

% Calculate Filter Order (n) and Cutoff (Wn)
[n, Wn] = buttord(Wp, Ws, Rp, Rs);

fprintf('Filter Order: %d\n', n);
fprintf('Normalized Cutoff Frequency: %.4f\n', Wn);

fprintf('2) Done.\n')

% 3) Calculate the filter coefficients and apply them to the signal, i.e.,
% filter the signal. Plot the filtered signal into the same figure as the
% original signal. Make sure to add a title, the axis descriptions, and a
% legend.

fprintf('3) Start.\n')

% Calculate coefficients
[b, a] = butter(n,Wn);

% Apply the filter to the raw signal
signal_filtered = filter(b, a, signal);

% Plot filtered signal in the same figure
subplot(2,1,2);
plot(time, signal_filtered, 'r', 'LineWidth', 1.5);
title('Filtered Signal');
xlabel('Time (s)');
ylabel('Amplitude (V)');
legend('Filtered Output');
grid on;

fprintf('3) Done.\n')

%% 4. Fourier Transform
% Calculate the single-sided Fourier transform of the filtered signal.

fprintf('4) Start.\n')

% 4.1) First, obtain the length of the original and filtered signal and 
% calculate their means. Print both mean values.

L = length(signal);
mean_raw = mean(signal);
mean_filt = mean(signal_filtered);

fprintf('Mean Original: %.4f\n', mean_raw);
fprintf('Mean Filtered: %.4f\n', mean_filt);

% 4.2) Do the FFT for both signals; see the docs and lecture slides for
% help. Make sure to remove the mean from the signals.

% 4.2.1.) Original signal

Y_raw = fft(signal - mean_raw);
P2_raw = abs(Y_raw/L);
P1_raw = P2_raw(1:floor(L/2)+1);
P1_raw(2:end-1) = 2*P1_raw(2:end-1);
 
% 4.2.2) Filtered signal

Y_filt = fft(signal_filtered - mean_filt);
P2_filt = abs(Y_filt/L);
P1_filt = P2_filt(1:floor(L/2)+1);
P1_filt(2:end-1) = 2*P1_filt(2:end-1);

% Frequency Vector for X-axis
f = Fs * (0:(L/2)) / L;

% 4.2.3) When plotting, only visualize the spectrum of to 500 Hz.

figure(2)

plot(f, P1_raw, 'b'); hold on;
plot(f, P1_filt, 'r', 'LineWidth', 1.5);
xlim([0 500]); % Limit X-axis to 500 Hz
title('Single-Sided Amplitude Spectrum');
xlabel('Frequency (Hz)');
ylabel('Amplitude |P1(f)|');
legend('Original (Noisy)', 'Filtered (Clean)');
grid on;

fprintf('4) Done.\n')

%% 5. Frequency Identification
% Write a function that automatically detects a signals frequency based
% on its frequency spectrum. You can assume there's only a single signal
% and noise has been removed. The function must return the amplitude and
% the frequency of this signal.

fprintf('5) Start.\n')

% 5.2) What is the frequency of they signal you have analyzed?

[amp_detected, freq_detected] = identify_frequency(f, P1_filt);

fprintf('Detected Frequency: %.2f Hz\n', freq_detected);
fprintf('Detected Amplitude: %.2f V\n', amp_detected);

fprintf('5) Done.\n')

% 5.1) Define function

function [amplitude, frequency] = identify_frequency(f_vec, p1_vec)
    % Find the maximum value (peak) in the spectrum
    [max_val, max_idx] = max(p1_vec);
    
    amplitude = max_val;
    frequency = f_vec(max_idx);
end
 