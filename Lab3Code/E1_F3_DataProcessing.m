%% 
%{
ABOUT: Process data saved from Simulink model
DATE: February 25, 2022
VERSION: 1.0

LICENSE: Copyright Leonam Pecly, Keyvan Hashtrudi-Zaad and Queen's University. MREN 348: Introduction to Robotics is available under an
Ontario Commons License (https://vls.ecampusontario.ca/wp-content/uploads/2021/01/Ontario-Commons-License-1.0.pdf).
Third-party copyright material is not considered part of the project for the purposes of licensing.
%}
%%
clear; clc;

fname = 'E1_Data.mat';
A = load(fname); % Load experiment data to variable A (struct variable)

%% Load QArm parameters and maps Phi to Theta
QArm_Parameters;
sample_rate = 10;
i_s = 100;
i_e = round(length(A.q.signals.values)*0.95/sample_rate)*sample_rate-1;

q = A.q.signals.values(i_s:i_e,1:4);            % Load Phi data
theta = q - [0, pi/2-QArm.beta, QArm.beta, 0];    % Maps Phi to Theta

%% Computes velocity (Theta_d) and acceleration (Theta_dd)

for k = 1:4 % For each joint
    pos_raw = downsample(theta(:,k),sample_rate);
    pos_raw_interp = interp(pos_raw,sample_rate);

    [p,v,a] = f_PVA_CD(pos_raw_interp,A.Ts,50,24);
    pos(:,k)    = p;      % Position
    vel(:,k)    = v;      % Velocity
    accel(:,k)  = a;      % Acceleration
end

V_signals = [vel(:,1) .* vel(:,2),...
             vel(:,1) .* vel(:,3),...
             vel(:,1) .* vel(:,4),...
             vel(:,2) .* vel(:,3),...
             vel(:,2) .* vel(:,4),...
             vel(:,3) .* vel(:,4),...
             ];

%% Compute Torque

Current = A.Current.signals.values(i_s:i_e,1:4);
for k = 1:length(Current)   
    % QArm motors torque
    Torque_Experiment(k,:) = f_QArm_Torque(Current(k,:));          
    
    % Computer Matrices M, V (Combination of Va and Vb), and G
    [M, Va, Vb, G] = f_QArm_Dynamics(q(k,:), 0);
    
    % Torque from the QArm model (A.11)
    Torque_Lagrangian(k,:) = M * [accel(k,:)]' ...          
                            + Va * [V_signals(k,:)]'...     
                            + Vb * [vel(k,:).^2]'...
                            + G;
end

% Remaining torque, i.e. commanded to the motors (experiment) minus ...
% the torque known from the QArm model (equation (A.11))
Torque_f = Torque_Experiment - Torque_Lagrangian; %  This torque will be used in the identification

%% Identification using BLS method
clear PHI y;

for k = 1 : length(pos) % For every observation 'k'
   
   phi_f(:,1) = [vel(k,1); 0; 0; 0];                             % theta_dot_1
   phi_f(:,2) = [sign(vel(k,1)); 0; 0; 0];                       % sign(theta_dot_1)
   phi_f(:,3) = [0; vel(k,2); 0; 0];                             % theta_dot_2
   phi_f(:,4) = [0; sign(vel(k,2)); 0; 0];                       % sign(theta_dot_2)
   phi_f(:,5) = [0; 0; vel(k,3); 0];                             % theta_dot_3
   phi_f(:,6) = [0; 0; sign(vel(k,3)); 0];                       % sign(theta_dot_3)
   phi_f(:,7) = [0; 0; 0; vel(k,4)];                             % theta_dot_4
   phi_f(:,8) = [0; 0; 0; sign(vel(k,4))];                       % sign(theta_dot_4)


   %% Add the regresion matrix phi to matrix PHI (all observations)
   k1 = (k-1)*4 +1;
   k2 = k1+3;
   PHI(k1:k2,:) = phi_f;
   y(k1:k2,1) = [Torque_f(k,1); Torque_f(k,2); Torque_f(k,3); Torque_f(k,4)];
end
Identified_Parameters = (PHI'*PHI)\PHI'*y; % BLS method - equation (3.12)
save('E1_Parameters','Identified_Parameters');

%% Estimated torque
torque_temp = PHI*Identified_Parameters;
for k = 1 : length(pos)
    k1 = (k-1)*4 +1;
    k2 = k1+3;
    Torque_f_hat(k,1:4) = torque_temp(k1:k2,:);
end

%% Remaning torque
Torque_e = Torque_f-Torque_f_hat;

%% Figure 1: Plots of Torque_f vs Joint Velocity
figure(1); hold off;
for k = 1 : 4
    subplot(2,2,k); hold off
    plot(vel(:,k),Torque_f(:,k),':');
    grid on; grid minor;
    title(['Joint ',mat2str(k)]);
    xlabel('Velocity [rad/s]');
    ylabel('Torque [N]');
end

%% Figure 2: Plots of Torque_Error vs Joint Velocities
figure(2); hold off;
for k = 1 : 4
    subplot(2,2,k); hold off;
    plot(vel(:,k), Torque_f_hat(:,k), 'b.', 'DisplayName', 'Estimated Friction Torque');
    grid on; grid minor;
    title(['Joint ', num2str(k), ' Friction Profile']);
    xlabel('Velocity [rad/s]');
    ylabel('Torque [Nm]');
    legend;
end


%% Function to compute Velocity and Central Difference

function [pos,vel,accel] = f_PVA_CD(position,dt,freq_hz,order)
Gs_ = tf([1],[1/(2*pi*freq_hz) 1]);
Gs = Gs_^order;
Gz = c2d(Gs,dt,'tustin');

A = cell2mat(Gz.num);
B = cell2mat(Gz.den);

POS(:,1) = position(:,1);

VEL_(:,1) = f_diff(position(:,1),2)/(2*dt);
VEL(:,1) = filtfilt(A,B,VEL_(:,1));

ACCEL_(:,1) = f_diff(VEL_(:,1),2)/(2*dt);
ACCEL(:,1) = filtfilt(A,B,ACCEL_(:,1));

pos(:,1)  = POS(1:end,1);
vel(:,1)  = [0; VEL(1:end,1); 0];
accel(:,1) = [0;  0; ACCEL(1:end,1); 0; 0];
end