%% 
%{
ABOUT: To validate identification
DATE: February 25, 2022
VERSION: 1.0

LICENSE: Copyright Leonam Pecly, Keyvan Hashtrudi-Zaad and Queen's University. MREN 348: Introduction to Robotics is available under an
Ontario Commons License (https://vls.ecampusontario.ca/wp-content/uploads/2021/01/Ontario-Commons-License-1.0.pdf).
Third-party copyright material is not considered part of the project for the purposes of licensing.
%}
%%
clear; clc;

%% Load Identified Parameters and QArm Experimental Data
fname = 'E1_Parameters.mat';
load(fname);

fname = 'QArm_CircleYZ.mat';
load(fname);

%% Torque from QArm Model

for k = 1:length(q)   
    % Computer Matrices M, V (Combination of Va and Vb), and G
    [M, Va, Vb, G] = f_QArm_Dynamics(q(k,:), 0);
    
    % Torque from the QArm model (A.11)
    Torque_Lagrangian(k,:) = M * [accel(k,:)]' ...          
                         + Va * [V_signals(k,:)]'...     
                         + Vb * [vel(k,:).^2]'...
                         + G;
end

%% Torque from Identified Parameters
clear PHI;

for k = 1 : length(vel) % For every observation 'k'
   
   %% Regression matrix phi_f
   phi_f(:,1) = [vel(k,1); 0; 0; 0];                             % theta_dot_1
   phi_f(:,2) = [sign(vel(k,1)); 0; 0; 0];                       % sign(theta_dot_1)
   phi_f(:,3) = [0; vel(k,2); 0; 0];                             % theta_dot_2
   phi_f(:,4) = [0; sign(vel(k,2)); 0; 0];                       % sign(theta_dot_2)
   phi_f(:,5) = [0; 0; vel(k,3); 0];                             % theta_dot_3
   phi_f(:,6) = [0; 0; sign(vel(k,3)); 0];                       % sign(theta_dot_3)
   phi_f(:,7) = [0; 0; 0; vel(k,4)];                             % theta_dot_4
   phi_f(:,8) = [0; 0; 0; sign(vel(k,4))];                       % sign(theta_dot_4)


   %% Add the regresion matrix PHI to matrix PHI (all observations)
   k1 = (k-1)*4 +1;
   k2 = k1+3;
   PHI(k1:k2,:) = phi_f;
end

clear torque_temp;
torque_temp = PHI*Identified_Parameters;

for k = 1 : length(vel)
    k1 = (k-1)*4 +1;
    k2 = k1+3;
    Torque_Parameters(k,1:4) = torque_temp(k1:k2,:);
end

%% Compute all Total Torque (Model + Parameters)
Torque_Total = Torque_Lagrangian + Torque_Parameters;

%% %RMSE computation
t_k1 = 902;
t_k2 = 4902;

for k = 1:4 % For each joint
    pRMSE_Lagrangian(:,k) = 100 * rms(Torque_Experiment(t_k1:t_k2,k)-Torque_Lagrangian(t_k1:t_k2,k))/rms(Torque_Experiment(t_k1:t_k2,k));
    pRMSE_Total(:,k) = 100 * rms(Torque_Experiment(t_k1:t_k2,k)-Torque_Total(t_k1:t_k2,k))/rms(Torque_Experiment(t_k1:t_k2,k));
end

%% Creating Figure
figure(3); hold off;
for k = 1:4 % For each joint
    subplot(4,1,k); hold off;
    plot(Time,Torque_Experiment(:,k)); hold on
    plot(Time,Torque_Lagrangian(:,k)); hold on
    plot(Time,Torque_Total(:,k)); hold on
    grid on;
    title(['Joint ',mat2str(k)]);
    ylabel('Torque [N]');
    if (k==4)
        xlabel('Time [s]');
    end
    xlim([0.5 10.5]);
end
legend('Experiment','Lagrangian','Total');

