%% 
%{
ABOUT: Initialize parameters of QArm.
DATE: February 25, 2022
VERSION: 1.0

LICENSE: Copyright Leonam Pecly, Keyvan Hashtrudi-Zaad and Queen's University. MREN 348: Introduction to Robotics is available under an
Ontario Commons License (https://vls.ecampusontario.ca/wp-content/uploads/2021/01/Ontario-Commons-License-1.0.pdf).
Third-party copyright material is not considered part of the project for the purposes of licensing.
%}
%%
QArm.dt = 0.002;

QArm.L1 = 0.1400;
QArm.L2 = 0.3500;
QArm.L3 = 0.0500;
QArm.L4 = 0.2500;
QArm.L5 = 0.1500;
QArm.beta = atan(QArm.L3/QArm.L2);

QArm.l2 = sqrt(QArm.L2^2 + QArm.L3^2);
QArm.l3 = QArm.L4 + QArm.L5;
%%

QArm.eta = [272.5 272.5 272.5 353.5];
QArm.Km = [0.00884 0.00884 0.00884 0.005];
QArm.Rm = [2.72 2.72/2 2.72 5.21];
QArm.Kb = [0.014 0.014 0.014 0.007];

QArm.Pgain = 2048/pi * 1/128 * 1/885;
QArm.Kp_PositionMode = [800 800 800 800].*QArm.Pgain;