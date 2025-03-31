% This function sets all the required parameters and state-space representation of the 
% SRV02 Flexible Joint System

function [Jarm, K_Stiff] = config_flexjoint(SPRING, HUB_POSITION, ARM_POSITION, ARM_LOAD)

% SPRINGS
% spring 1 used, LEAST STIFFNESS
aL(1) = 0.0254*1.0;
aFr(1) = 0.85;
aK(1) = 0.22*1000*.85;
% spring used E0240-026-1250 MEDIUM STIFFNESS (DEFAULT)
aL(2) = 0.0254*1.25;
aFr(2) = 1.33;
aK(2) = 0.368*1000*.85;
% spring used E0240-029-1250 STIFFEST
aL(3) = 0.0254*1.25;
aFr(3) = 1.78;
aK(3) = 0.665*1000*.85;

% POSITION ON HUB (BASE)
ad(1) = 0.0254*1.25; % from hinge to spring insertion on plate, along arm
ad(2) = 0.0254*1.00; % from hinge to spring insertion on plate, along arm
ad(3) = 0.0254*0.75; % from hinge to spring insertion on plate, along arm

% POSITION ON ARM (LINK)
aR(1) = 0.0254*4.0; % from hinge to spring insertion on arm
aR(2) = 0.0254*3.5; % from hinge to spring insertion on arm
aR(3) = 0.0254*3.0; % from hinge to spring insertion on arm

r = 0.0254*1.25; % Fixed distance

% Setting to current hub position
if strcmp (HUB_POSITION, 'A')
    d = ad(1);
elseif strcmp (HUB_POSITION, 'B')
    d = ad(2);
elseif strcmp (HUB_POSITION, 'C')
    d = ad(3);
end

% Setting to current arm position
R = aR(ARM_POSITION);

% Setting to current spring
L = sqrt((R-d)^2 + r^2);%aL(SPRING);
Fr = aFr (SPRING);
K = aK(SPRING);

K_Stiff = 2*R*Fr*(d/L + (r/L)^2*(K*R/Fr - R/L));

% Calculating Arm Inertia
m_main_arm = 0.064; % 64 grams
l_main_arm = 11.75 * 0.0254; % 12 inches
J_main_arm = m_main_arm*l_main_arm^2 / 3; % J = m*l^2 / 3

m_short_arm = 0.03; % 30 grams
l_short_arm = 6 * 0.0254; % 6 inches
J_short_arm = m_short_arm*l_short_arm^2 / 12; % J = m*l^2 / 12 - This is the short arm's J about its center of gravity.

D_short_arm (1) = 9 * 0.0254; % 9 inches away from rotation axis
D_short_arm (2) = 10 * 0.0254; % 10 inches away from rotation axis
D_short_arm (3) = 11 * 0.0254; % 11 inches away from rotation axis

% The Complete arm inertia is calculated using the parallel axis theorem. J = J_cg + M*D^2
if ARM_LOAD == 0
    Jarm = J_main_arm;
else
    Jarm = J_main_arm + J_short_arm + m_short_arm * (D_short_arm(ARM_LOAD))^2;
end