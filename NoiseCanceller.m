%Ava Mehri, UIC, amehri2@uic.edu, 2021
%Noise canceller
%an adoptive noise canceller, Wiener filter, steepest descent algorithm to remove Gaussian random noises from audio signals.

clear all
close all

% mp3signal = audioread('Abr.mp3');
% info = audioinfo('Abr.mp3');
% Fs = info.SampleRate;
% audiowrite('Abr.wav', mp3signal, Fs);

wavSignal = audioread('Abr.wav');
[l,q] = size(wavSignal);
signal = wavSignal(:,2);

signal = signal(1:1400000);
[l,q] = size(signal);

wavinfo = audioinfo('Abr.wav');
Fs = wavinfo.SampleRate;
% sound(signal,Fs);
var = 0.1;
noise = sqrt(var)*randn(l,1);

% Additive noise
noisySig = signal + noise;
% multiplicative noise
% noisySig = signal .* (noise+1);
audiowrite('noisy_Abr.wav', noisySig, Fs);

n = 100;   %number of coefficients (5,20,50,100)
i = l; 
k = 10000; %iterations

p = zeros(n,1);
p(1) = var;

R = xcorr(noise,noise,n-1,'unbiased'); %cross-correlation matrix for noise 
R = R(n:(2*n-1)); 

R = toeplitz(R);  %R size of nxn (autocorrelation matrix )
w0 = (R^-1)*p;          %optimum wiener filter vector
    
e = zeros(i,1);  %initial values for error signal
s = [0; noise];         
w1 = repmat(-1,n,1);   %Initial values (Wiener Coefficients)
m = 0.001;              %Steepest-Descent step
wt = zeros([n,i]); 
wt(:,1) = w0;  

for i = n+1:i
  w1 = w1 + m*(p-R*w1);    
  wt(:,i) = w1;
  e(i) = s(i:-1:i-(n-1))' * w0;   
end
e = [e(2:i);0];

denoised = noisySig-e;           
%sound(noisySig,Fs);     
%sound(denoised,Fs);
audiowrite('denoised_Abr.wav', denoised, Fs);

remainedNoise = denoised - signal;
% remainedNoise = (denoised ./ signal) -1;
SNR_noisy = sum(signal.^2)/sum(noise.^2)
SNR_denoised = sum(signal.^2)/sum(remainedNoise.^2)

X=fft(noisySig);  
figure 
x=linspace(0,2*pi,i); 
plot(x,abs(X)) 
title('fft of noisy signal')

Y=fft(denoised);  
figure 
x=linspace(0,2*pi,i); 
plot(x,abs(Y)) 
title('fft of denoised signal')