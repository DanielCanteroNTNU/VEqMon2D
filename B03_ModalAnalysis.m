function [Veh] = B03_ModalAnalysis(Veh)

% Performing modal analysis of vehicle

% ---- Input ----
% Veh = Structure variable with field
%   .Sys = which contains the system matrices (M,C,K)
% ---- Output ----
% Veh = Additional information in the structure variable
%   .Modal.w = ciruclar frequency
%   .Modal.f = Frequencies (Hz)
%   .Modal.num_modes = number of modes
%   .Modal.Mode(i).values = Mode of vibration of mode-i
% ----------------

% Eingevalue analysis
[modes,aux1] = eig(Veh.SysM.K,Veh.SysM.M);
aux1 = diag(aux1);

% Vehicle circuar frequencies
Veh.Modal.w = sqrt(aux1);
% Vehicle frequecies (Hz)
Veh.Modal.f = Veh.Modal.w/(2*pi);
% Number of modes
Veh.Modal.num_modes = length(Veh.Modal.w);
% Modes of vibration
for i = 1:Veh.Modal.num_modes
    Veh.Modal.Mode(i).values = modes(:,i);
end % for i = 1:length(Veh.Modal.w)

% ---- End of function ----