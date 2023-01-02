% Validation of a the derived equations of motion by calculating its modal 
% properties and comparing them to the modal analysis of a separately 
% developed vehicle model in Abaqus.

% *************************************************************************
% *** Script part of VEqMon2D tool for Matlab environment.              ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***
% ***                                                                   ***
% *** If you found this tool useful, please cite:                       ***
% *** D. Cantero. VEqMon2D - Equations of motion generation tool of 2D  ***
% *** vehicles with Matlab, SoftwareX, Volume 19, 2022.                 ***
% ***   DOI: https://doi.org/10.1016/j.softx.2022.101103                ***
% ***                                                                   ***
% *************************************************************************

% Vehicle model: Vehicle_3A3_2_G_1_2_3_1_1

% The frequencies have been calculated using the Abaqus model
%   Abq_Vehicle_3A3_2_G_1_2_3_1_1.inp
%
% The results of the modal analysis in Abaqus are (in Hz):
%     1.4010
%     1.5923
%     1.9731
%     4.8321
%     7.3248
%     8.8112
%     12.351
%     12.367
%     12.661
%     13.422
%     29.174
%     29.775

clear; clc;

% -- Saved results --
Calc.Load.path = 'Results\';
Calc.Load.function_name = 'Vehicle_3A3_2_G_1_2_3_1_1';

% -- Vehicle information -- (as found in Vehicle_3A3_2_G_1_2_3_1_1.m)
num_axles_per_body = [3  3  2];
num_axles_per_group = [1  2  3  1  1];
with_articulation = [1  0];
 
% -- Vehicle variables -- (as found in Vehicle_3A3_2_G_1_2_3_1_1.m)
% Veh.Prop.mBi = [[mB1, mB2, mB3]]
% Veh.Prop.IBi = [[IB1, IB2, IB3]]
% Veh.Prop.kSj = [[kS1, kS2, kS3, kS4, kS5]]
% Veh.Prop.cSj = [[cS1, cS2, cS3, cS4, cS5]]
% Veh.Prop.mGj = [[mG1, mG2, mG3, mG4, mG5]]
% Veh.Prop.IGj = [[IG1, IG2, IG3, IG4, IG5]]
% Veh.Prop.kTk = [[kT1, kT2, kT3, kT4, kT5, kT6, kT7, kT8]]
% Veh.Prop.cTk = [[cT1, cT2, cT3, cT4, cT5, cT6, cT7, cT8]]
% Veh.Prop.ai = [[a1, a2, a3]]
% Veh.Prop.bi = [[b1, b2]]
% Veh.Prop.dj = [[d1, d2, d3, d4, d5]]
% Veh.Prop.ek = [[e1, e2, e3, e4, e5, e6, e7, e8]]

% Corresponding numerical values:
Veh.Prop.mBi = [5e3, 30e3, 20e3];           % [kg]
Veh.Prop.IBi = [4e3, 20e3, 10e3];           % [kg*m2]
Veh.Prop.kSj = [0.5, 2, 3, 1, 1]*1e6;       % [N/m]
Veh.Prop.cSj = [1, 2, 3, 1, 1]*1e3;         % [N*s/m]
Veh.Prop.mGj = [1, 2, 3, 1, 1]*750;         % [kg]
Veh.Prop.IGj = [0, 2, 3, 0, 0]*1e2;         % [kg*m2]
Veh.Prop.kTk = [1.75, 3.5, 3.5, 3.5, 3.5, ...
    3.5, 3.5, 3.5]*1e6;                     % [N/m]
Veh.Prop.cTk = [1, 1, 1, 1, 1, 1, 1, 1]*1e4;% [N*s/m]
Veh.Prop.ai = [0, 6, 4];                    % [m]
Veh.Prop.bi = [2, 5];                       % [m]
Veh.Prop.dj = [-1, 3, 2, -2.5, 2.5];        % [m]
Veh.Prop.ek = [0, -1, 1, -1.2, 0, 1.2, 0,0];% [m]

% -- System matrices --
Calc.Load.original_path = cd(Calc.Load.path);
eval(['[Veh] = ',Calc.Load.function_name,'(Veh);'])
cd(Calc.Load.original_path);

% -- Modal analysis --
[Veh] = B03_ModalAnalysis(Veh);

disp('The natural frequencies are:')
disp(round(Veh.Modal.f,3))

% ---- End of script ----
