% Simulation of of a vehicle traveling at speed v over a irregularity profile

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

% -- Vehicle to run --
Calc.Load.path = 'results\';
Calc.Load.function_name = 'Vehicle_3A3_2_G_1_2_3_1_1';
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

% Vehicle speed
Veh.Pos.vel = 20;       % [m/s]

% -- Irregularity profile --
Calc.Profile.L = 100;           % Length of profile [m]
Calc.Profile.dx = 0.01;         % Sampling distance [m]
Calc.Profile.Step.x = 50;       % Location of step [m]
Calc.Profile.Step.h = 0.01;     % Height of step [m]

% -- Solver --
Calc.Solver.dt = 0.001;         % Time step [s]

% ---------------------------- Calculations -------------------------------

% -- Vehicle's system matrices --
Calc.Load.original_path = cd(Calc.Load.path);
eval(['[Veh] = ',Calc.Load.function_name,'(Veh);'])
cd(Calc.Load.original_path);

% -- Auxiliary variables --
% Number of vehicle axles
Veh.Prop.num_axles = length(Veh.Prop.ax_sp);
% Vehicle wheelbase and number of wheels
Veh.Prop.wheelbase = Veh.Prop.ax_dist(end);
% Time solver array
Calc.Solver.t = 0:Calc.Solver.dt:(Calc.Profile.L-Veh.Prop.wheelbase)/Veh.Pos.vel;
% Number of solver steps
Calc.Solver.num_t = length(Calc.Solver.t);
% Gavity
Calc.Cte.grav = -9.81;   % [m/s^2]
% Calculation of static deformation
Veh.Static.disp = (Veh.Static.F_vector_no_grav*Calc.Cte.grav)/Veh.SysM.K;
% Static contact forces
Veh.Static.load = Veh.Prop.kTk'.*(Veh.SysM.N2w*Veh.Static.disp');

% -- Profile calculations --
% Profile space discretization
Calc.Profile.x = 0:Calc.Profile.dx:Calc.Profile.L;
% Number of samples
Calc.Profile.num_x = length(Calc.Profile.x);
% Profile elevation
Calc.Profile.h = zeros(1,Calc.Profile.num_x);
Calc.Profile.h(Calc.Profile.x>=Calc.Profile.Step.x) = Calc.Profile.Step.h;

% -- Profile for each wheel --
Veh.Pos.wheels_x = ones(Veh.Prop.num_axles,1)*(Calc.Solver.t*Veh.Pos.vel) + Veh.Prop.wheelbase;
for axle_num = 2:Veh.Prop.num_axles
    Veh.Pos.wheels_x(axle_num,:) = Veh.Pos.wheels_x(axle_num,:) - Veh.Prop.ax_dist(axle_num);
end % for axle_num = 2:Veh(veh_num).Prop.num_axles
% Initialize
Veh.Pos.wheels_h = Veh.Pos.wheels_x*0;
Veh.Pos.wheels_hd = Veh.Pos.wheels_h;
% Interpotlation from full length profile 
%interp_method = 'linear';   % Matlab's default (leads to high frequency artifacts)
interp_method = 'pchip';    % Piecewise cubic interpolation
for axle_num = 1:Veh.Prop.num_axles
    Veh.Pos.wheels_h(axle_num,:) = interp1(Calc.Profile.x,Calc.Profile.h,Veh.Pos.wheels_x(axle_num,:),interp_method);
end % for axle_num = 1:Veh.Prop.num_axles
Veh.Pos.wheels_hd = [zeros(Veh.Prop.num_axles,1),diff(Veh.Pos.wheels_h,1,2)./...
    (ones(Veh.Prop.num_axles,1)*(diff(Veh.Pos.wheels_x(1,:),1,2)./Veh.Pos.vel))];
% % Graphical check
% figure; counter = 0;
% for axle_num = 1:Veh.Prop.num_axles
%     counter = counter + 1;
%     subplot(2,Veh.Prop.num_axles,counter);
%     plot(Veh.Pos.wheels_x(axle_num,:),Veh.Pos.wheels_h(axle_num,:));
%     axis tight; xlabel('Distance (m)'); ylabel('Elevantion (m)'); title(['Axle ',num2str(axle_num)]);
% end % for axle_num = 1:Veh.Prop.num_axles
% for axle_num = 1:Veh.Prop.num_axles
%     counter = counter + 1;
%     subplot(2,Veh.Prop.num_axles,counter);
%     plot(Veh.Pos.wheels_x(axle_num,:),Veh.Pos.wheels_hd(axle_num,:));
%     axis tight; xlabel('Distance (m)'); ylabel('First time derivative (m/s)'); title(['Axle ',num2str(axle_num)]);
% end % for axle_num = 1:Veh.Prop.num_axles

% -- Time integration --
[Sol] = B04_TimeIntegration(Calc,Veh);

% -- Graphical results --

% All displacements of the independent DOFs
figure;
for DOF_num = 1:Veh.DOF(1).num_independent
    subplot(Veh.DOF(1).num_independent,1,DOF_num);
    plot(Calc.Solver.t,Sol.Veh.U(DOF_num,:));
    ylabel(['DOF ',num2str(DOF_num)]);
    axis tight;
    if DOF_num == 1
        title('Responses of each independent DOF')
    elseif DOF_num == Veh.DOF(1).num_independent
        xlabel('Time (s)');
    end % if DOF_num == 1
end % for DOF_num = 1:Veh.DOF(1).num_independent

% % Displacements of all the axles
% figure;
% for axle_num = 1:Veh.Prop.num_axles
%     subplot(Veh.Prop.num_axles,1,axle_num);
%     plot(Calc.Solver.t,Sol.Veh.Wheels.U(axle_num,:));
%     axis tight; ylabel(['Axle ',num2str(axle_num)]);
%     if axle_num == 1
%         title('Axle displacements')
%     elseif axle_num == Veh.Prop.num_axles
%         xlabel('Time (s)');
%     end % if axle_num == 1
% end % for axle_num = 1:Veh.Prop.num_axles

% % Contact forces of all axles
% figure;
% Sol.Veh.F_contact = diag(Veh.Prop.kTk)*(Sol.Veh.Wheels.U-Veh.Pos.wheels_h) + ...
%     diag(Veh.Prop.cTk)*(Sol.Veh.Wheels.V-Veh.Pos.wheels_hd);
% Sol.Veh.F_contact = Sol.Veh.F_contact + Veh.Static.load*ones(1,Calc.Solver.num_t);
% for axle_num = 1:Veh.Prop.num_axles
%     subplot(Veh.Prop.num_axles,1,axle_num)
%     plot(Calc.Solver.t,Sol.Veh.F_contact(axle_num,:));
%     axis tight; ylabel(['Axle ',num2str(axle_num)]);
%     if axle_num == 1
%         title('Contact forces of all axles')
%     elseif axle_num == Veh.Prop.num_axles
%         xlabel('Time (s)');
%     end % if axle_num == 1
% end % for axle_num = 1:Veh.Prop.num_axles

% Cleaning workspace
clear axle_num interp_method counter axle_num DOF_num

% ---- End of script ----
