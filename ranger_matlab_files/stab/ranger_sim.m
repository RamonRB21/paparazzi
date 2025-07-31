clear all
close all
clc

sample_time = 1/500;   % Define the sample time for discrete systems

% Define proportional and derivative gain vectors
Kp = [4.1089; 4.7304; 7.36];
Kd = [10; 18; 18];

% Define time constants for filters
am = 0.025;
as = 0.08;

% Create discrete-time transfer functions for motor and servo dynamics
Amd=tf(am,[1 am-1],sample_time);
Asd=tf(as,[1 as-1],sample_time);

% Convert discrete-time transfer functions to continuous time using Tustin method
Amc=d2c(Amd,'tustin');
Asc=d2c(Asd,'tustin');

H_cutoff_freq = 5;   % Define cutoff frequency for Butterworth filter (Hz)

% Create a 2nd-order Butterworth low-pass filter in continuous time
[Hc_num,Hc_den] = butter(2,2*pi*H_cutoff_freq,'s');
Hc = tf(Hc_num,Hc_den);
[Ahc,Bhc,Chc,Dhc] = ssdata(Hc);

% Initialize block diagonal state-space matrices for 7 repeated systems
Ah7 = [];
Bh7 = [];
Ch7 = [];
Dh7 = [];

% Initialize block diagonal state-space matrices for 3 repeated systems
Ah3 = [];
Bh3 = [];
Ch3 = [];
Dh3 = [];

% Loop to build 7x block diagonal filters
for k = 1:7
    Ah7 = blkdiag(Ah7, Ahc);
    Bh7 = blkdiag(Bh7, Bhc);
    Ch7 = blkdiag(Ch7, Chc);
    Dh7 = blkdiag(Dh7, Dhc);
end

% Loop to build 3x block diagonal filters
for k = 1:3
    Ah3 = blkdiag(Ah3, Ahc);
    Bh3 = blkdiag(Bh3, Bhc);
    Ch3 = blkdiag(Ch3, Chc);
    Dh3 = blkdiag(Dh3, Dhc);
end

% Discretize the continuous Butterworth filter using Tustin method
Hd = c2d(Hc,sample_time,'tustin');
H_discr_num = cell2mat(Hd.num);   % Extract numerator coefficients
H_discr_den = cell2mat(Hd.den);   % Extract denominator coefficients

% Define control allocation matrix G and its pseudo-inverse
G = [-10.67  10.67  0     0     0     0 0;
      8      8     -22.22 15.62 0     0 0;
      0      0      0     0     26.67 0 0;
     -0.847 -0.847 -0.847 0     0     0 0;
      0      0      0    -0.002 0     0 0]./1000;
G_inv = pinv(G);   % Compute Moore-Penrose pseudoinverse

% Define thrust allocation matrix
G_thrust = zeros(1,7);
G_thrust = [0 0 0 0 0 0 0;
            0 0 0 0 0 0 0;
            0 0 0 0 0 0 0;
            -0.847 -0.847 -0.847 0 0 0 0;
            0 0 0 -0.002 0 0 0]./1000;

% Obtain discrete-time linearized model
[Ad,Bd,Cd,Dd]=dlinmod('bf_ranger_indi');
Ranger_INDI=ss(Ad,Bd,Cd,Dd,sample_time);
Ranger_INDIr=minreal(Ranger_INDI);
damp(Ranger_INDIr);

% Plot pole-zero map of discrete-time system
figure(1)
pzmap(Ranger_INDIr)
grid on

% Obtain continuous-time linearized model of Simulink model
[A,B,C,D]=linmod('bf_ranger_indi_continu');
Ranger_INDIc=ss(A,B,C,D);
Ranger_INDIcr=minreal(Ranger_INDIc);
damp(Ranger_INDIcr);

% Plot pole-zero map of continuous-time system
figure(2)
pzmap(Ranger_INDIcr)
grid on

% Plot singular values of both discrete and continuous systems
figure(3)
hold on
sigma(Ranger_INDIr)
sigma(Ranger_INDIcr)
grid on
hold off