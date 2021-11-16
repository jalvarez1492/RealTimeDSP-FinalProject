function [output] = hsfilter(theta,Fs, input)
%HSFILTER 
% This function is taken from the book DAFX - Digital Audio Effects, 
% Chapter 6 Section 3. 
% This function implements the hsfilter function found in DAFX book in
% section 6.3.

theta = theta+90;
theta0 = 150;
alfa_min = 0.05;

c = 334; % speed of sound in meters / sec
a = 0.08; % radius of head in meters
w0 = c/a;

alfa = 1 + alfa_min/2 + (1-alfa_min/2)*cos(theta/ theta0*pi);

% feed-forward terms
B = [(alfa + w0/Fs)/(1 + w0/Fs), (-alfa + w0/Fs)/(1 + w0/Fs)];
% feedback terms
A = [1, -(1 - w0/Fs)/(1 + w0/Fs)];

if (abs(theta) < 90)
    gdelay = -Fs/w0 * (cos(theta*pi/180) - 1);
else
    gdelay = Fs/w0 * ((abs(theta)-90)*pi/180 + 1);
end

a = (1 - gdelay) / (1 + gdelay); % allpass filter coeffecient

out_magn = filter(B, A, input);

output = filter([a,1], [1,a], out_magn);

plot

end