close all
clear all
clc

Ts = 1/500;

pos_global = readtable("test_dyn_ranger_actuators.csv");

% Define time window
t_start = 9.5;
t_end = 10.5;

% Filter rows where Time is within the interval
pos = pos_global(pos_global{:,2} >= t_start & pos_global{:,2} <= t_end, :);

% Compute angle from coordinates (projected on XY plane)
t = pos{:,2} - t_start;
xa = pos.Unlabeled1133_1;
ya = pos.Unlabeled1133_2;
[xac,yac] = fit_circle(xa,ya);
xm = pos.Unlabeled1134_1;
ym = pos.Unlabeled1134_2;
[xmc,ymc] = fit_circle(xm,ym);

xa = xa - xac;
ya = ya - yac;
xm = xm - xmc;
ym = ym - ymc;

thetaa = atan2(ya,xa)*180/pi;
thetaa = thetaa - mean(thetaa(1:100));
thetam = atan2(ym,xm)*180/pi;
thetam = thetam - mean(thetam(1:100));

figure
plot(xa,ya)
title("Aileron deflection Read Data")
xlabel("Y Raw Coordinate [mm]")
ylabel("Z Raw Coordinate [mm]")
axis equal
grid minor
figure
plot(xm,ym)
title("Motor Tilt Read Data")
xlabel("Y Raw Coordinate [mm]")
ylabel("Z Raw Coordinate [mm]")
axis equal
grid minor

%% TIME CONSTANT

y_final = mean(thetaa(end-100:end));  % estimate steady-state
y_63 = 0.632 * y_final;

delay = 0.38;

% Find the time when output reaches 63.2% of final value
idx = find(thetaa >= y_63, 1, 'first');
t_63 = t(idx) - delay;

tau = t_63;  % first-order time constant
a = 2*Ts/(2*tau + Ts);
f_act = -log(1-a)/Ts;

fprintf('Estimated tau (ailerons) = %.4f s\n', tau);
fprintf('Estimated actuator frequency (ailerons) = %.4f Hz\n\n', f_act);

% Define first-order system with delay
G = tf(y_final, [tau 1], 'InputDelay', delay);

% Create step input (unit step)
u = ones(size(t));

% Simulate response
y_model = lsim(G, u, t);

% Plot
figure;
plot(t, thetaa, 'b', 'DisplayName', 'Measured signal');
hold on;
plot(t, y_model, 'r--', 'LineWidth', 1.5, 'DisplayName', sprintf('1st-order (\\tau = %.2f s)', tau));
xlabel('Time [s]');
ylabel('Angle [rad]');
legend('Location', 'best');
grid on;
title(sprintf('1st-Order Fit with Delay = %.2f s', delay));

abs_error = abs(thetaa - y_model);
rmse = sqrt(mean((thetaa - y_model).^2));

% === Relative Error ===
rel_error = abs_error ./ max(abs(thetaa));
rel_error_percent = 100 * rel_error;

% === Statistics ===
mean_abs_error = mean(abs_error);
mean_rel_error_percent = mean(rel_error_percent);

% --- DISPLAY ---
fprintf('Mean Absolute Error: %.4f\n', mean_abs_error);
fprintf('Mean Relative Error: %.2f %%\n', mean_rel_error_percent);
fprintf('RMSE Error: %.4f\n\n', rmse);

% === Plot results ===
figure;
subplot(2,1,1);
plot(t, abs_error, 'b');
ylabel('Absolute Error');
title('Absolute Error vs Time');
yline(mean_abs_error,'-','Mean');
grid on;

subplot(2,1,2);
plot(t, rel_error_percent, 'r');
ylabel('Relative Error (%)');
xlabel('Time [s]');
title('Relative Error vs Time');
yline(mean_rel_error_percent,'-','Mean');
grid on;

%% RATE LIMIT

% Step 1: Define the rising interval manually
t_start = 0.4;
t_end = 0.63;

% Step 2: Select the portion of the data within that interval
idx = (t >= t_start) & (t <= t_end);
t_rising = t(idx);
y_rising = thetam(idx);

% Step 3: Perform linear fit to estimate the slope
p = polyfit(t_rising, y_rising, 1);  % Linear fit: y = p(1)*t + p(2)
slope = p(1);  % This is the slope

% Display the result
fprintf('Estimated rate limit (tilt) in [%.2f s, %.2f s]: %.4f rad/s\n', t_start, t_end, slope*pi/180);
fprintf('Estimated rate limit (tilt) in [%.2f s, %.2f s]: %.4f PPRZ/loop cycle\n', t_start, t_end, slope*8700/90*Ts);

% Step 4: Plot the result
figure;
plot(t, thetam, 'b', 'DisplayName', 'Measured signal');
hold on;
plot(t_rising, polyval(p, t_rising), 'r--', 'LineWidth', 2, 'DisplayName', sprintf('Slope = %.2f deg/s', slope));
xlabel('Time [s]');
ylabel('Angle [rad]');
legend('Location', 'best');
title('Slope estimation in rising segment');
grid on;