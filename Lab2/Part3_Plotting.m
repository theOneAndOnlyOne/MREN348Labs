% Load MAT files (modify filenames if needed)
data = load('withoutAvoidance.mat'); % Change filename as necessary

% Extract relevant data
q_data = data.q_data;         % Joint angles over time
q_dot_data = data.q_dot_data; % Joint velocities over time
state_data = data.state_data; % State transitions over time
x_data = data.x_data;         % End-effector positions
xd_data = data.xd_data;       % Desired end-effector positions

% Extract time vectors (assuming the first column is time)
time = q_data(:,1);

% Plot Joint Angles
figure;
plot(time, q_data(:,2:end), 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Joint Angles (rad)');
title('Joint Angles Over Time');
legend('q1', 'q2', 'q3', 'q4', 'Location', 'best');
grid on;

% Plot Joint Velocities
figure;
plot(time, q_dot_data(:,2:end), 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Joint Velocities (rad/s)');
title('Joint Velocities Over Time');
legend('q̇1', 'q̇2', 'q̇3', 'q̇4', 'Location', 'best');
grid on;

% Plot End-Effector Position
figure;
plot3(x_data(:,2), x_data(:,3), x_data(:,4), 'r', 'LineWidth', 1.5);
hold on;
plot3(xd_data(:,2), xd_data(:,3), xd_data(:,4), 'b--', 'LineWidth', 1.5);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('End-Effector Position vs Desired');
legend('Actual Position', 'Desired Position');
grid on;
hold off;

% Plot State Transitions
figure;
plot(time, state_data(:,2), 'k', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('State');
title('State Transitions Over Time');
grid on;
