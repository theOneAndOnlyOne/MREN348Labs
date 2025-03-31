% This function sets all the required parameters and state-space representation of the 
% SRV02 Flexible Link System

function [Jarm, K_Stiff, K_GAGE] = config_flexgage_exp( )

Link_L = 0.0254 * 15; % Length of the Link is 15 inches
Link_M = 0.065; % 65 grams
Wc = 2 * pi * 3; % Natural Frequency was experimentally determined to be 3 Hz
K_GAGE = 1 / 19;

Jarm = Link_M * Link_L ^ 2 / 3; % Calculte the Moment of Inertia of a Link (Assumed Rigid)
K_Stiff = Wc^2 * Jarm; % Estimate the Stiffness of the simplified Link Model
