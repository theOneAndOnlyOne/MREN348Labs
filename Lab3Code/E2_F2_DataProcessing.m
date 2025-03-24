%% 
%{
ABOUT: Process data saved from Simulink Model
DATE: February 25, 2022
VERSION: 1.0

LICENSE: Copyright Leonam Pecly, Keyvan Hashtrudi-Zaad and Queen's University. MREN 348: Introduction to Robotics is available under an
Ontario Commons License (https://vls.ecampusontario.ca/wp-content/uploads/2021/01/Ontario-Commons-License-1.0.pdf).
Third-party copyright material is not considered part of the project for the purposes of licensing.
%}
%%
clear; clc;

fname = 'E2_Data.mat';
A = load(fname); % Load experiment data to variable A (struct variable)

%% Time of Point A
time_A = 11.5;                                 % Seconds

%%
QArm_Parameters;    
i_s = time_A/QArm.dt;                        % Start index of Point A
i_e = i_s + 5/QArm.dt;         

q = A.q.signals.values(i_s:i_e,1:4);                % Load q data
theta = q - [0, pi/2-QArm.beta, QArm.beta, 0];      % Maps q to theta

q_deg = rad2deg(q);
theta_deg = rad2deg(theta);
%% Compute Experiment Torque

Current = A.Current.signals.values(i_s:i_e,1:4);
for k = 1:length(Current)   
    % QArm motors torque
    Torque_Experiment(k,:) = f_QArm_Torque(Current(k,:));          
end

%% Compute F_ext

for k = 1:length(q) % For every sample time
    % Computer Matrices M, V (Combination of Va and Vb), and G
    [~, ~, ~, G] = f_QArm_Dynamics(q(k,:), 0);    % Gravitational vector
    J = f_QArm_DiffKinematics(q(k,:));       % Jacobian
    
    %% Fext computation
    Torque_Langrangian(k,:) = G;                     % Since joint velocities and accelerations are close to zero
    %
    Fext(k,:) = pinv(J')*(Torque_Experiment(k,:) - Torque_Langrangian(k,:))';  % → F_ext = (τ - G) * inv(J^T)  
    
    

    


    %%

end

%%
Fext_mean = mean(Fext)
Fext_STD = std(Fext)
