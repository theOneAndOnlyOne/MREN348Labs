function [M, Va, Vb, G] = f_QArm_Dynamics(phi, pointLoadMass)
%% 
%{
ABOUT: Function with QArm Dynamics
DATE: February 25, 2022
VERSION: 1.0

LICENSE: Copyright Leonam Pecly, Keyvan Hashtrudi-Zaad and Queen's University. MREN 348: Introduction to Robotics is available under an
Ontario Commons License (https://vls.ecampusontario.ca/wp-content/uploads/2021/01/Ontario-Commons-License-1.0.pdf).
Third-party copyright material is not considered part of the project for the purposes of licensing.

Acknowledgement: Based on code provided by Quanser Consulting Inc. as
supporting documentation.
%}
%%
g = 9.80665;

%% Manipulator parameters:
L1 = 0.1400;
L2 = 0.3500;
L3 = 0.0500;
L4 = 0.2500;
L5 = 0.1500;
beta = atan(L3/L2);

%% Alternate parameters
l1 = L1;                        % lambda 1
l2 = sqrt(L2^2 + L3^2);         % lambda 2
l3 = L4 + L5;                   % lambda 3
theta(1) = phi(1);
theta(2) = phi(2) + beta - pi/2;
theta(3) = phi(3) - beta;
theta(4) = phi(4);

%% Dynamic Parameters
m1 = 0.7906;
m2 = 0.4591;
m3 = 0.269;
m4 = 0.257;
mL = pointLoadMass;
I1A = 1.489e-3;
I1L = 1;
I2A = 1.922e-4;
I2L = 9.61e-3;
I3A = 2.679e-4;
I3L = 2.069e-3;
I4A = 5.528e-4;
I4L = 1.12e-3;

lc1 = 0.0399;
lc2 = 0.1071;
lc3 = 0.1561;
lc4 = 0.0998;

%% Dynamic Matrices
% Inertia Matrix
M11 = I1A + m2*(l2 - lc2)^2*cos(theta(2))^2 + I2A*sin(theta(2))^2 + I2L*cos(theta(2))^2 + m3*l2^2*cos(theta(2))^2 + m3*lc3^2*sin(theta(2)+theta(3))^2 - 2*m3*l2*lc3*cos(theta(2))*sin(theta(2)+theta(3))...
    + I3L*sin(theta(2)+theta(3))^2 + I3A*cos(theta(2)+theta(3))^2 + m4*(l3 - lc4)^2*sin(theta(2)+theta(3))^2 + m4*l2^2*cos(theta(2))^2 - 2*m4*(l3-lc4)*l2*cos(theta(2))*sin(theta(2)+theta(3))...
    + I4L*sin(theta(2)+theta(3))^2 + I4A*cos(theta(2)+theta(3))^2 + mL*l3^2*sin(theta(2)+theta(3))^2 + mL*l2^2*cos(theta(2))^2 - 2*mL*l2*l3*cos(theta(2))*sin(theta(2)+theta(3));
M14 = -I4A*cos(theta(2)+theta(3));
M22 = m2*(l2-lc2)^2 + I2L + m3*l2^2+m3*lc3^2 - 2*m3*l2*lc3*sin(theta(3)) + I3L + m4*(l2+l3-lc4)^2 + I4L + mL*l2^2 + mL*l3^2 - 2*mL*l2*l3*sin(theta(3));
M23 = m3*lc3^2 - m3*lc3*l2*sin(theta(3)) + I3L + m4*(l3 - lc4 - l2*sin(theta(3)))*(l3 - lc4) + I4L - mL*(l3^2 - l2*l3*sin(theta(3)));
M32 = m3*lc3^2 - m3*lc3*l2*sin(theta(3)) + I3L + m4*(l3 - lc4 - l2*sin(theta(3)))*(l3 - lc4) + I4L - mL*(l3^2 - l2*l3*sin(theta(3)));
M33 = m3*lc3^2 + I3L + m4*(lc4 - l3)^2 + I4L + mL*l3^2;
M41 = -I4A*cos(theta(2)+theta(3));
M44 = I4A;
M = [M11 + 272.5^2*0.65e-5/2, 0, 0, M14; 0, M22 + 272.5^2*0.8e-05/2, M23, 0; 0, M32, M33 + 0.8e-05/4 * 272.5^2, 0; M41, 0, 0, M44 + 0.1e-07 * 353.5^2];

% Coriolis Matrix
B11 = -2*m2*(l2-lc2)^2*sin(theta(2))*cos(theta(2)) + 2*I2A*sin(theta(2))*cos(theta(2)) - 2*I2L*sin(theta(2))*cos(theta(2)) - 2*m3*l2^2*sin(theta(2))*cos(theta(2))...
     + 2*m3*lc3^2*sin(theta(2)+theta(3))*cos(theta(2)+theta(3)) + 2*m3*l2*lc3*sin(theta(2))*sin(theta(2)+theta(3)) - 2*m3*l2*lc3*cos(theta(2))*cos(theta(2)+theta(3))...
     + 2*I3L*sin(theta(2)+theta(3))*cos(theta(2)+theta(3)) - 2*I3A*sin(theta(2)+theta(3))*cos(theta(2)+theta(3)) + 2*m4*(l3 - lc4)^2*sin(theta(2)+theta(3))*cos(theta(2)+theta(3))...
     - 2*m4*l2^2*sin(theta(2))*cos(theta(2)) + 2*m4*(l3 - lc4)*l2*sin(theta(2))*sin(theta(2)+theta(3)) - 2*m4*(l3 - lc4)*l2*cos(theta(2))*cos(theta(2)+theta(3))...
     + 2*I4L*sin(theta(2)+theta(3))*cos(theta(2)+theta(3)) - 2*I4A*sin(theta(2)+theta(3))*cos(theta(2)+theta(3)) + 2*mL*l3^2*sin(theta(2)+theta(3))*cos(theta(2)+theta(3))...
     - 2*mL*l2^2*sin(theta(2))*cos(theta(2)) + 2*mL*l2*l3*sin(theta(2))*sin(theta(2)+theta(3)) - 2*mL*l2*l3*cos(theta(2))*cos(theta(2)+theta(3));
B12 =  2*m3*lc3^2*sin(theta(2)+theta(3))*cos(theta(2)+theta(3)) - 2*m3*l2*lc3*cos(theta(2))*cos(theta(2)+theta(3)) + 2*I3L*sin(theta(2)+theta(3))*cos(theta(2)+theta(3))...
     - 2*I3A*sin(theta(2)+theta(3))*cos(theta(2)+theta(3)) + 2*m4*(l3-lc4)^2*sin(theta(2)+theta(3))*cos(theta(2)+theta(3)) - 2*m4*(l3-lc4)*l2*cos(theta(2))*cos(theta(2)+theta(3))...
     + 2*I4L*sin(theta(2)+theta(3))*cos(theta(2)+theta(3)) - 2*I4A*sin(theta(2)+theta(3))*cos(theta(2)+theta(3)) + 2*mL*l3^2*sin(theta(2)+theta(3))*cos(theta(2)+theta(3))...
     - 2*mL*l2*l3*cos(theta(2))*cos(theta(2)+theta(3));
B15 =  I4A*sin(theta(2)+theta(3));
B16 =  I4A*sin(theta(2)+theta(3));
B23 = -I4A*sin(theta(2)+theta(3));
B24 = -2*m3*l2*lc3*cos(theta(3)) - 2*mL*l2*l3*cos(theta(3));
B33 = -I4A*sin(theta(2)+theta(3));
B41 =  I4A*sin(theta(2)+theta(3));
B42 =  I4A*sin(theta(2)+theta(3));
Va = [B11, B12, 0, 0, B15, B16; 0, 0, B23, B24, 0, 0; 0, 0, B33, 0, 0, 0; B41, B42, 0, 0, 0, 0];

% Centrifugal Matrix
C21 = -(-m2*(l2-lc2)^2*sin(theta(2))*cos(theta(2)) + I2A*sin(theta(2))*cos(theta(2)) - I2L*sin(theta(2))*cos(theta(2)) - m3*l2^2*sin(theta(2))*cos(theta(2))...
     + m3*lc3^2*sin(theta(2)+theta(3))*cos(theta(2)+theta(3)) + m3*l2*lc3*sin(theta(2))*sin(theta(2)+theta(3)) - m3*l2*lc3*cos(theta(2))*cos(theta(2)+theta(3))  ...
     + I3L*sin(theta(2)+theta(3))*cos(theta(2)+theta(3)) - I3A*sin(theta(2)+theta(3))*cos(theta(2)+theta(3)) + m4*(l3-lc4)^2*sin(theta(2)+theta(3))*cos(theta(2)+theta(3))...
     - m4*l2^2*sin(theta(2))*cos(theta(2)) + m4*(l3-lc4)*l2*sin(theta(2))*sin(theta(2)+theta(3)) - m4*(l3-lc4)*l2*cos(theta(2))*cos(theta(2)+theta(3)) ...
     + I4L*sin(theta(2)+theta(3))*cos(theta(2)+theta(3)) - I4A*sin(theta(2)+theta(3))*cos(theta(2)+theta(3)) + mL*l3^2*sin(theta(2)+theta(3))*cos(theta(2)+theta(3))...
     - mL*l2^2*sin(theta(2))*cos(theta(2)) + mL*l2*l3*sin(theta(2))*sin(theta(2)+theta(3)) - mL*l2*l3*cos(theta(2))*cos(theta(2)+theta(3)));
C31 = -(m3*lc3^2*sin(theta(2)+theta(3))*cos(theta(2)+theta(3)) - m3*l2*lc3*cos(theta(2))*cos(theta(2)+theta(3)) + I3L*sin(theta(2)+theta(3))*cos(theta(2)+theta(3))...
      - I3A*sin(theta(2)+theta(3))*cos(theta(2)+theta(3)) + m4*(l3-lc4)^2*sin(theta(2)+theta(3))*cos(theta(2)+theta(3)) ...
      - m4*(l3-lc4)*l2*cos(theta(2))*cos(theta(2)+theta(3)) + I4L*sin(theta(2)+theta(3))*cos(theta(2)+theta(3)) - I4A*sin(theta(2)+theta(3))*cos(theta(2)+theta(3))... 
      + mL*l3^2*sin(theta(2)+theta(3))*cos(theta(2)+theta(3)) - mL*l2*l3*cos(theta(2))*cos(theta(2)+theta(3)));
C32 = m3*l2*lc3*cos(theta(3)) + mL*l2*l3*cos(theta(3));
Vb = [0, 0, 0, 0; C21, 0, 0, 0; C31, C32, 0, 0; 0, 0, 0, 0];

% Gravity Matrix

G2 = -g*(m2*(l2 - lc2)*cos(theta(2)) + m3*(l2*cos(theta(2)) - lc3*sin(theta(2)+theta(3))) + m4*(l2*cos(theta(2)) - (l3 - lc4)*sin(theta(2)+theta(3))) + mL*(l2*cos(theta(2)) - l3*sin(theta(2)+theta(3))) );
G3 = g*(m3*lc3*sin(theta(2)+theta(3)) + m4*(l3 - lc4)*sin(theta(2)+theta(3)) + mL*l3*sin(theta(2)+theta(3)) );

G = [0; G2; G3; 0];