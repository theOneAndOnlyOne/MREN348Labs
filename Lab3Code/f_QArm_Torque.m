function [Torque] = f_QArm_Torque(Current)
%% 
%{
ABOUT: Function to compute torque using current
DATE: February 25, 2022
VERSION: 1.0

LICENSE: Copyright Leonam Pecly, Keyvan Hashtrudi-Zaad and Queen's University. MREN 348: Introduction to Robotics is available under an
Ontario Commons License (https://vls.ecampusontario.ca/wp-content/uploads/2021/01/Ontario-Commons-License-1.0.pdf).
Third-party copyright material is not considered part of the project for the purposes of licensing.
%}
%% Joint 1 to 3
for k = 1 : 3
    Current_k = Current(1,k);
    
    Torque(1,k) =   + 0.0179 * abs(Current_k).^6 ...
                    - 0.2409 * abs(Current_k).^5 ...
                    + 1.2595 * abs(Current_k).^4 ...
                    - 3.1846 * abs(Current_k).^3 ...
                    + 3.7674 * abs(Current_k).^2 ...
                    + 0.7867 * abs(Current_k);
            
    if (Current_k < 0)
        Torque(1,k) = -Torque(1,k);
    end
end
%% Joint 4
k = 4;
Current_k = Current(1,k);
    
Torque(1,k) =  + 0.1163 * abs(Current_k).^2 ...
               + 1.4485 * abs(Current_k);

if (Current_k < 0)
    Torque(1,k) = -Torque(1,k);
end

end
