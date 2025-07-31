clear all
close all
clc

sample_time = 1/500;   % Define the sample time for discrete systems

% Define time constants for filters
am = 0.025;
as = 0.08;

% Create discrete-time transfer functions for motor and servo dynamics
Amd=tf(am,[1 am-1],sample_time);
Asd=tf(as,[1 as-1],sample_time);

% Convert discrete-time transfer functions to continuous time using Tustin method
Amc=d2c(Amd,'tustin');
Asc=d2c(Asd,'tustin');

%% STABILIZATION LOOP DATA
% Define proportional and derivative gain vectors
Kps = [4.1089; 4.7304; 7.36];
Kds = [10; 18; 18];

Hs_cutoff_freq = 5;   % Define cutoff frequency for Butterworth filter (Hz)

% Create a 2nd-order Butterworth low-pass filter in continuous time
[Hcs_num,Hcs_den] = butter(2,2*pi*Hs_cutoff_freq,'s');
Hcs = tf(Hcs_num,Hcs_den);
[Ahcs,Bhcs,Chcs,Dhcs] = ssdata(Hcs);

% Initialize block diagonal state-space matrices for 7 repeated systems
Ahs7 = [];
Bhs7 = [];
Chs7 = [];
Dhs7 = [];

% Initialize block diagonal state-space matrices for 3 repeated systems
Ahs3 = [];
Bhs3 = [];
Chs3 = [];
Dhs3 = [];


% Loop to build 7x block diagonal filters
for k = 1:7
    Ahs7 = blkdiag(Ahs7, Ahcs);
    Bhs7 = blkdiag(Bhs7, Bhcs);
    Chs7 = blkdiag(Chs7, Chcs);
    Dhs7 = blkdiag(Dhs7, Dhcs);
end

% Loop to build 3x block diagonal filters
for k = 1:3
    Ahs3 = blkdiag(Ahs3, Ahcs);
    Bhs3 = blkdiag(Bhs3, Bhcs);
    Chs3 = blkdiag(Chs3, Chcs);
    Dhs3 = blkdiag(Dhs3, Dhcs);
end

% Define control allocation matrix G and its pseudo-inverse
Gs = [-10.67  10.67  0     0     0     0 0;
      8      8     -22.22 15.62 0     0 0;
      0      0      0     0     26.67 0 0;
     -0.847 -0.847 -0.847 0     0     0 0;
      0      0      0    -0.002 0     0 0]./1000;
Gs_inv = pinv(Gs);   % Compute Moore-Penrose pseudoinverse

% Define thrust allocation matrix
Gs_thrust = zeros(1,7);
Gs_thrust = [0 0 0 0 0 0 0;
            0 0 0 0 0 0 0;
            0 0 0 0 0 0 0;
            -0.847 -0.847 -0.847 0 0 0 0;
            0 0 0 -0.002 0 0 0]./1000;
        
%% GUIDANCE LOOP DATA 
% Define proportional and derivative gain vectors
Kpg = [0.581; 0.581; 0.5943];
Kdg = [2.0; 2.0; 1.8];

Hg_cutoff_freq = 5;   % Define cutoff frequency for Butterworth filter (Hz)

% Create a 2nd-order Butterworth low-pass filter in continuous time
[Hcg_num,Hcg_den] = butter(2,2*pi*Hg_cutoff_freq,'s');
Hcg = tf(Hcg_num,Hcg_den);
[Ahcg,Bhcg,Chcg,Dhcg] = ssdata(Hcg);

% Initialize block diagonal state-space matrices for 3 repeated systems
Ahg3 = [];
Bhg3 = [];
Chg3 = [];
Dhg3 = [];


% Loop to build 3x block diagonal filters
for k = 1:3
    Ahg3 = blkdiag(Ahg3, Ahcg);
    Bhg3 = blkdiag(Bhg3, Bhcg);
    Chg3 = blkdiag(Chg3, Chcg);
    Dhg3 = blkdiag(Dhg3, Dhcg);
end

%% LOOP RUN
         
% Obtain linearized model
[A,B,C,D]=linmod('guid_stab_loop');
Ranger_INDIc=ss(A,B,C,D);
Ranger_INDIcr=minreal(Ranger_INDIc);
damp(Ranger_INDIcr);

% Plot pole-zero map of system
figure(1)
pzmap(Ranger_INDIcr)
grid on

% Obtain linearized model
[A,B,C,D]=linmod('guid_stab_loop_no_rot');
Ranger_INDIcnr=ss(A,B,C,D);
Ranger_INDIcrnr=minreal(Ranger_INDIcnr);
damp(Ranger_INDIcrnr);

% Plot pole-zero map of system
figure(2)
pzmap(Ranger_INDIcrnr)
grid on

% Plot singular values of both systems
figure(3)
hold on
sigma(Ranger_INDIcr)
sigma(Ranger_INDIcrnr)
grid on
hold off
