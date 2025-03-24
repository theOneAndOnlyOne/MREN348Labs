%% 
%{
ABOUT: Initialization for Simulink model
DATE: February 25, 2022
VERSION: 1.0

LICENSE: Copyright Leonam Pecly, Keyvan Hashtrudi-Zaad and Queen's University. MREN 348: Introduction to Robotics is available under an
Ontario Commons License (https://vls.ecampusontario.ca/wp-content/uploads/2021/01/Ontario-Commons-License-1.0.pdf).
Third-party copyright material is not considered part of the project for the purposes of licensing.
%}
%%
clear; clc;

%% QArm Parameters
QArm_Parameters
Torque2Voltage = (1./QArm.eta) .* (QArm.Rm./QArm.Km);

%% Control Settings
wn = [6.67 6.67 6.67 6.67];      % rad/s for all joints
zeta = 1;                        % critically damped

%% Partitioning Approach
Ieff = [0.3535 0.4879 0.1814 0.0018];    % average inertia
%Beff = [0.391289313652276 0.035039848412252 0.581548559565145 0.467168851420579 0.427067089186307 0.423011580099028 0.047575535897681 0.005487681044332];                       % update with friction if needed
Beff = [0.391289313652276 0.581548559565145 0.427067089186307 0.047575535897681];                       % update with friction if needed

Kp = wn*wn';    % [15.72 21.71 8.07 0.08]
Kd = 2*zeta*wn;    % [4.71 6.51 2.42 0.02]

