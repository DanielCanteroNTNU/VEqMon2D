function [Sol] = B04_TimeIntegration(Calc,Veh)

% Performs the time intregration of vehicle system traversing an irregularity profile

% -------------------------------------------------------------------------
% ---- Input ----
% Calc = Structure with Calculation variables, including at least:
%   .Solver.dt = Time step
%   .Solver.num_t = Total number of time steps
%   .Solver.t = Array of time steps to compute
% Veh = Structure with Vehicle's variables, including at least:
%   .DOF.num_independent = number of independent DOF
%   .Prop.num_axles = number of axles
%   .Prop.kTk = array with tyre suspension stiffness
%   .Pos.wheels_h = elevation of irregularity for each axle and time step
%   .SysM.M = Vehicle global Mass matrix
%   .SysM.C = Vehicle global Damping matrix
%   .SysM.K = Vehicle global Stiffness matrix
% ---- Output ----
% Sol = Addition of fields to structure Sol:
%   .Veh.U = Results for DOFs displacement
%   .Veh.V = Results for DOFs velocities
%   .Veh.V = Results for DOFs accelerations
%   .Veh.Wheels.U = Vertical displacement for each axle
%   .Veh.Wheels.Urel = Relative vertical displacement for each axle
%   .Veh.Wheels.V = Vertical velocities for each axle
%   .Veh.Wheels.Vrel = Relative vertical velocities for each axle
% -------------------------------------------------------------------------

% -- Initialize variables --
Sol.Veh.U = zeros(Veh.DOF(1).num_independent,Calc.Solver.num_t);
Sol.Veh.A = Sol.Veh.U;
Sol.Veh.V = Sol.Veh.U;
Sol.Veh.Wheels.U = zeros(Veh.Prop.num_axles,Calc.Solver.num_t);
Sol.Veh.Wheels.Urel = Sol.Veh.Wheels.U;
Sol.Veh.Wheels.V = Sol.Veh.Wheels.U;
Sol.Veh.Wheels.Vrel = Sol.Veh.Wheels.U;

% Newmark-beta parameters
Calc.Solver.NewMark.delta = 0.5; 
Calc.Solver.NewMark.beta = 0.25;

% -- Initial static response of vehicle --
aux1 = Veh.SysM.N2w'*(Veh.Prop.kTk'.*Veh.Pos.wheels_h(:,1));
Sol.Veh.U(:,1) = Veh.SysM.K\aux1;

% ---- Dynamic integration calculation ----

% -- Effective stiffness matrix --
eff_K_veh = Veh.SysM.K + Veh.SysM.M/(Calc.Solver.NewMark.beta*Calc.Solver.dt^2) + ...
    Calc.Solver.NewMark.delta/(Calc.Solver.NewMark.beta*Calc.Solver.dt)*Veh.SysM.C;

% -- Force Matrix --
F_ext = (Veh.Prop.kTk'*ones(1,Calc.Solver.num_t)).*Veh.Pos.wheels_h + ...
        (Veh.Prop.cTk'*ones(1,Calc.Solver.num_t)).*Veh.Pos.wheels_hd;
F_ext = Veh.SysM.N2w'*F_ext;

% -- Step by step calculation --
for t = 1:Calc.Solver.num_t-1

    % ---- Vehicle System ----
    % Newmark-beta scheme (As seen in B014)
    A = Sol.Veh.U(:,t)/(Calc.Solver.NewMark.beta*Calc.Solver.dt^2) + ...
            Sol.Veh.V(:,t)/(Calc.Solver.NewMark.beta*Calc.Solver.dt) + ...
            Sol.Veh.A(:,t)*(1/(2*Calc.Solver.NewMark.beta)-1);
    B = (Calc.Solver.NewMark.delta/(Calc.Solver.NewMark.beta*Calc.Solver.dt)*Sol.Veh.U(:,t) - ...
            (1-Calc.Solver.NewMark.delta/Calc.Solver.NewMark.beta)*Sol.Veh.V(:,t) - ...
            (1-Calc.Solver.NewMark.delta/(2*Calc.Solver.NewMark.beta))*Calc.Solver.dt*Sol.Veh.A(:,t));
    Sol.Veh.U(:,t+1) = eff_K_veh\(F_ext(:,t+1) + Veh.SysM.M*A + Veh.SysM.C*B);
    Sol.Veh.V(:,t+1) = Calc.Solver.NewMark.delta/(Calc.Solver.NewMark.beta*Calc.Solver.dt)*Sol.Veh.U(:,t+1) - B;
    Sol.Veh.A(:,t+1) = Sol.Veh.U(:,t+1)/(Calc.Solver.NewMark.beta*Calc.Solver.dt^2) - A;

end % for t = 1:num_t_app-1

% -- Additional Output generation --
% Wheel displacements
Sol.Veh.Wheels.U = Veh.SysM.N2w*Sol.Veh.U;
% Relative wheel displacements
Sol.Veh.Wheels.Urel = Sol.Veh.Wheels.U - Veh.Pos.wheels_h;
% Wheel velocities
Sol.Veh.Wheels.V = Veh.SysM.N2w*Sol.Veh.V;
% Relative wheel velocities
Sol.Veh.Wheels.Vrel = Sol.Veh.Wheels.V - Veh.Pos.wheels_hd;

% ---- End of script ----