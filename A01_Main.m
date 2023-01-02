% Main scritp for VEqMon2D tool

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

clear; clc;

% -- Particular values --
% num_axles_per_body = Number of axles per body [Tractor, Trailer 1, Trailer 2,...]
% num_axles_per_group = Number of axles per group
% with_articulation = Array with 1/0 flag to indicate the presence of articulation

% % Initial example: Simple 2-axle truck
% Inputs.num_axles_per_body = 2;
% Inputs.num_axles_per_group = [1,1];
% Inputs.with_articulation = [];

% Validation example: Truck with 3 bodies
%   First body has 1 single axle and a tandem axle
%   Second body is articulated with body 1. It has also a tridem axle
%   Third body is a towed trailer with 2 individual axles
Inputs.num_axles_per_body = [3,3,2];
Inputs.num_axles_per_group = [1,2,3,1,1];
Inputs.with_articulation = [1,0];

% -- Saving options --
Save.on = 1;    % Save results into a script
Save.path_name = 'Results\';

% -- Calculations and function generation --
B01_calculations(Inputs,Save);

% ---- End of script ----
