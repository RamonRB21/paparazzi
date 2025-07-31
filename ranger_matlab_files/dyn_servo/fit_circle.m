function [x_c,y_c] = fit_circle(x,y)

% Step 1: Formulate the linear system
A = [-2*x, -2*y, ones(size(x))];
b = -(x.^2 + y.^2);

% Step 2: Solve the overdetermined system A * params = b
params = A \ b;

x_c = params(1);  % X coordinate of the circle center
y_c = params(2);  % Y coordinate of the circle center
r = sqrt(x_c^2 + y_c^2 - params(3));  % Estimated radius

% Display results
% fprintf('Estimated circle center: (%.4f, %.4f)\n', x_c, y_c);
% fprintf('Estimated radius: %.4f\n', r);

% Step 3: plot to visualize the result
theta_plot = linspace(0, 2*pi, 200);
x_circle = x_c + r * cos(theta_plot);
y_circle = y_c + r * sin(theta_plot);

figure;
plot(x, y, 'b.', 'DisplayName', 'Measured data');
hold on;
plot(x_circle, y_circle, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Fitted circle');
plot(x_c, y_c, 'ko', 'MarkerFaceColor', 'k', 'DisplayName', 'Estimated center');
axis equal;
grid minor;
xlabel('X [mm]');
ylabel('Y [mm]');
legend;
title('Circle fitting to XY data');

end
