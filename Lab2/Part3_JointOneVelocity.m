% List of MAT files
matFiles = {'withoutAvoidance.mat', '27cmAvoidance.mat', '30cmAvoidance.mat',};

% Define colors for plotting
colors = {'r', 'b', 'g', 'm'};
labels = {'No Avoidance', '27cm Avoidance', '30cm Avoidance', '35cm Avoidance'};

% Initialize figure for Joint 1 Velocity
figure; hold on;
title('Joint 1 Velocity Over Time');
xlabel('Time (s)');
ylabel('Joint 1 Velocity (rad/s)');
grid on;

for i = 1:length(matFiles)
    % Load data
    data = load(matFiles{i});

    % Extract time and Joint 1 velocity
    time = data.q_dot_data(:,1); % Time vector
    q1_velocity = data.q_dot_data(:,2); % Joint 1 velocity

    % Plot Joint 1 Velocity
    plot(time, q1_velocity, 'LineWidth', 1.5, 'Color', colors{i});
end

legend(labels, 'Location', 'best');
hold off;
