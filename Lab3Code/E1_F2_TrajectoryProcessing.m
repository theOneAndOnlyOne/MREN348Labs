%% 
%{
ABOUT: Process trajectory created to use in Simulink model
DATE: September 22, 2022
VERSION: 1.1

LICENSE: Copyright Leonam Pecly, Keyvan Hashtrudi-Zaad and Queen's University. MREN 348: Introduction to Robotics is available under an
Ontario Commons License (https://vls.ecampusontario.ca/wp-content/uploads/2021/01/Ontario-Commons-License-1.0.pdf).
Third-party copyright material is not considered part of the project for the purposes of licensing.
%}
%%
clear;
load E1_F1_Trajectory.mat

%% Initialization
p0 = [0.45, 0, 0.49, 0];
Ts = 0.002;
Ts_cmd = 0.02;

%%
Traj_temp = [Trajectory(:,1:3) Trajectory_J4(:,1) Trajectory(:,4)];

if ((norm(Traj_temp(end,1:4)-p0) > 0.001))
    Traj_temp = [Traj_temp; [p0,[Traj_temp(end,5)+Ts_cmd]]];
end
Traj_temp = downsample(Traj_temp,Ts_cmd/Ts);
Traj = repelem(Traj_temp,Ts_cmd/Ts,1); % Reduces the frequency to command

%%
Traj_EE.time = Traj(:,5);
Traj_EE.signals.values = Traj(:,1:3);
Traj_EE.signals.dimensions = 3;

Traj_J4.time = Traj(:,5);
Traj_J4.signals.values = Traj(:,4);
Traj_J4.signals.dimensions = 1;

%% Plot End-Efector in 3D
figure(1); hold off;
subplot(2,1,1); hold off
plot3(Trajectory(:,1),Trajectory(:,2),Trajectory(:,3),'.-');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('End-Effector Position');
grid on;
view([-70 -65 85]);

subplot(2,1,2); hold off
plot(Trajectory_J4(:,2),rad2deg(q));
xlabel('Time [s]');
ylabel('Angle [deg]');
title('Joint Position');
xlim([t(1) t(end)]);
grid on; grid minor;