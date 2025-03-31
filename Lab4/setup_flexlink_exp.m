%% SETUP_FLEXLINK_EXP
%
% Sets the parameters needed to run the Simulink/QUARC models supplied with 
% the Quanser Rotary Flexible Link experiment. NOTE: THIS SCRIPT APPLIES TO
% THE REAL HARDWARE, NOT TO THE QUANSER INTERACTIVE LABS VIRTUAL HARDWARE.
% 
clear;
%
%% SRV02 Configuration
% External Gear Configuration: set to 'HIGH' or 'LOW'
EXT_GEAR_CONFIG = 'HIGH';
% Encoder Type: set to 'E' or 'EHR'
ENCODER_TYPE = 'E';
% Is SRV02 equipped with Tachometer? (i.e. option T): set to 'YES' or 'NO'
TACH_OPTION = 'YES';
% Type of Load: 'NO_LOAD', 'DISC_LOAD', 'BAR_LOAD', 'SLIP_RING', 'ROTFLEX' or 'FLEX_LINK' 
LOAD_TYPE = 'FLEX_LINK';
% Universal Power Module (UPM) Type: set to 'UPM_2405', 'UPM_1503', or 'UPM_1503x2'
UPM_TYPE = 'UPM_2405';
%
%% Control specifications
% SRV02 Position Control specifications
% Settling time (s)
ts = 1.5;
% Settling time percentage (%)
c_ts = 0.04;
% Percentage overshoot (%)
PO = 5.0;
%    
%% Filter Parameters
% SRV02 High-pass filter in PD control used to compute velocity
% Cutoff frequency (rad/s)
wcf_1 = 2 * pi * 16;
wcf_2 = 2 * pi * 3;
%
%% System Parameters
% Sets model variables according to the user-defined servo configuration
[Rm, kt, km, Kg, eta_g, Beq, Jeq, eta_m, K_POT, K_TACH, K_ENC, VMAX_UPM, IMAX_UPM] = config_servo_exp(EXT_GEAR_CONFIG, ENCODER_TYPE, TACH_OPTION, UPM_TYPE, LOAD_TYPE);
% Calculates the inertia and stiffness of the Flexible Joint Module.
[Jl, ~, K_GAGE] = config_flexgage_exp();
% Resonant frequency of the flexible link
w_r = 2*pi*3;
% Stiffness of the flexible link
K_Stiff = w_r^2 * Jl;
%
%% System Model
beta_m = eta_m*eta_g*kt*Kg/Rm;
A = [zeros(2) eye(2);
     0, K_Stiff/Jeq, -(Beq + beta_m*km*Kg)/Jeq, 0;
     0, -K_Stiff*(Jeq + Jl)/(Jeq*Jl), (Beq + beta_m*km*Kg)/Jeq, 0];
B = beta_m/Jeq*[zeros(2,1); 1; -1];
%
%% Control Setup
control_mode = "LQR"; % Set to PDA, PDB or LQR
% PD CONTROLLERS
% Set the control gains for PD controller A
K_pdA = 0;
cA = 0;
% Set the control gains for PD Controller B
K_pdB = 0;
cB = 0;
% LQR Control
% Set Q and R matrices to get desired response.
Q = diag([160 60 0.25 1]);
R = 1;
[K_lqr,~,~] = lqr(A,B,Q,R);
%
switch control_mode
    case "PDA"
        K = [K_pdA 0 K_pdA*cA 0];
        disp('PD Controller A Gains:');
        disp(['Kp (Kpd)   = ' num2str(K_pdA)]);
        disp(['Kd (Kpd*c) = ' num2str(K_pdA*cA)]);
    case "PDB"
        K = [K_pdB K_pdB K_pdB*cB K_pdB*cB];
        disp('PD Controller B Gains:');
        disp(['Kp (Kpd)   = ' num2str(K_pdB)]);
        disp(['Kd (Kpd*c) = ' num2str(K_pdB*cB)]);
    case "LQR"
        K = K_lqr;
        disp('LQR Weighting Matrices:')
        disp('Q =');disp(Q);
        disp(['R = ' num2str(R)]);
        disp('LQR Controller Gain:')
        disp(['K_lqr = [' num2str(K_lqr) ']']);
    otherwise
        K = zeros(1,4);
        warning('Incorrect control mode setting');
end