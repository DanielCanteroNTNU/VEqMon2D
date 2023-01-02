function [] = B02_IndexedSymbVars(var_name,num_vars,varargin)

% Script to create as many indexed symbolic variables as desired starting with a given name
% Optional: all variables are then grouped into a vector with the name "var_name+i"

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

% ---- Inputs ----
% var_name = String with the root name of the variable
% num_vars = number of sybmolic variables to create
% -- Optional inputs --
% subscript_letter = String with letter for the subscript. If not defined subscript_letter = "i"
% num_deriv = What derivative is requested (0 = no derivative)
%   If num_deriv is diff than [], the related symbolic variables are NOT saved in "Veh.Prop"
% ---- Outputs ----
% no formal output. Variables are created directly into the caller's workspace
%   Creates the specified symbolic variables
%   Also groups them into the variable "Veh.Prop"
% ------------

if num_vars > 0

    % Index letter
    if nargin >= 3
        subscript_letter = varargin{1};
        if isempty(subscript_letter)
            subscript_letter = 'i';
        end % if isempty(subscript_letter)
    else
        subscript_letter = 'i';
    end % if nargin >= 3
    
    % Addition string for derivative variables
    add_string = '';
    if nargin >= 4
        num_deriv = varargin{2};
        if num_deriv > 0
            add_string = ['_',repmat('d',1,num_deriv)];
        end % if num_deriv == 0
    else
        num_deriv = [];
    end % if nargin == 4

    % Generating expression to evaluate
    string_of_vars = [];
    for var_num = 1:num_vars
        string_of_vars = [string_of_vars,' ',var_name,num2str(var_num),add_string];
    end % for var_num = 1:num_vars
    
    % Creating variables in base workspace
    string_to_run = ['syms ',string_of_vars,' real;'];
    evalin('caller',string_to_run);

    % Creating vector with variables in base workspace (if num_vars = 1)
	string_to_run = [var_name,subscript_letter,add_string,' = [',string_of_vars,'];'];
	evalin('caller',string_to_run);
    
    % Additional output
    if isempty(num_deriv)
        string_to_run = ['Veh.Prop.([''',var_name,''',''',subscript_letter,''']) = [',string_of_vars,'];'];
        evalin('caller',string_to_run);
    end % if num_deriv == 0

end % if num_vars > 0
% ---- End of function ----
