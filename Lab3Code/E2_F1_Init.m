%% 
%{
ABOUT: Initialize variables for Simulink Model
DATE: February 25, 2022
VERSION: 1.0

LICENSE: Copyright Leonam Pecly, Keyvan Hashtrudi-Zaad and Queen's University. MREN 348: Introduction to Robotics is available under an
Ontario Commons License (https://vls.ecampusontario.ca/wp-content/uploads/2021/01/Ontario-Commons-License-1.0.pdf).
Third-party copyright material is not considered part of the project for the purposes of licensing.
%}
%%
clear; clc;

%% Create empty variables

Traj_EE.time = 0;
Traj_EE.signals.values = [0 0 0];
Traj_EE.signals.dimensions = 3;

Traj_J4.time = 0;
Traj_J4.signals.values = 0;
Traj_J4.signals.dimensions = 1;
%%

    