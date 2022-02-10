function [] = B01_calculations(Inputs,Save)

% Peforms the symbolic calculations to generate the system matrices for the
% vehicle specified in by the variables defined in Input variable.
% Generates a function file with the results.

% ---------------
% ---- Input ----
% Inputs = Structure variable with the following fields
%   .num_axles_per_body = Number of axles per body [Tractor, Trailer 1, Trailer 2,...]
%   .num_axles_per_group = Number of axles per group
%   .with_articulation = Array with 1/0 flag to indicate the presence of articulation
% Save = Structure variable with the following fileds
%   .on = Logical flag to save (or not) the results into a function
%   .path_name = String with path name where to save the results
% ---- Output ---
% No explicit output. But results will be saved into a separate m-file/function
% ---------------

disp('Processing inputs ...');

% -- Inputs checks --
if sum(Inputs.num_axles_per_body)~=sum(Inputs.num_axles_per_group)
    error('Missmatch of number of axles and groups');
end % if sum(Inputs.num_axles_per_body)~=sum(Inputs.num_axles_per_group)
if (length(Inputs.num_axles_per_body)-1)~=length(Inputs.with_articulation)
    error('Error in length of the array: Inputs.with_articulation');
end % if (length(Inputs.num_axles_per_body)-1)~=length(Inputs.with_articulation)

% -- Auxiliary values --
num_bodies = length(Inputs.num_axles_per_body);                        % Number of bodies
num_axles_per_body_longer_cumsum = [0,cumsum(Inputs.num_axles_per_body)];
% Body axles indices
for body_num = 1:num_bodies
    Body(body_num).axle_inds = (num_axles_per_body_longer_cumsum(body_num)+1):num_axles_per_body_longer_cumsum(body_num+1);
    Body(body_num).num_axles = length(Body(body_num).axle_inds);
end % for body_num = 1:num_bodies
clear num_axles_per_body_longer_cumsum body_num
num_groups = length(Inputs.num_axles_per_group);                       % Number of Groups
% Body groups indices
aux1 = 1;
group_i_is_in_body = [];
for body_num = 1:num_bodies
    Body(body_num).group_inds = (aux1:find(cumsum(Inputs.num_axles_per_group)==Body(body_num).axle_inds(end)));
    Body(body_num).num_groups = length(Body(body_num).group_inds);
    aux1 = Body(body_num).group_inds(end) + 1;
    group_i_is_in_body = [group_i_is_in_body,ones(1,Body(body_num).num_groups)*body_num];
end % for body_num = 1:num_bodies
clear aux1 body_num
num_axles = sum(Inputs.num_axles_per_body);                            % Total number of axles
num_groups_with_theta = sum(Inputs.num_axles_per_group>1);             % Number of groups that have more than one axle, thus need theta DOF

% -- Default values --
if ~isfield(Inputs,'with_articulation')
    Inputs.with_articulation = zeros(1,max(num_bodies-1,1));
end % if ~isfield(Inputs,'with_articulation')
num_articulations = num_bodies-1;

% Auxiliary variable for articulation defintion
Art.Body(1).back = 0;   % Default value (In case there is only 1 body)
Art.Body(1).front = 0;
for body_num = 1:num_bodies-1
    Art.Body(body_num).back = Inputs.with_articulation(body_num);
    Art.Body(body_num+1).front = Inputs.with_articulation(body_num);
end % for body_num = 1:num_bodies
Art.Body(num_bodies).back = 0;      % Additional defintions for scripting purposes

% Each axle i belongs to a group:
axle_i_is_in_group = [];
counter = 0;
for group_num = 1:num_groups
    counter = counter + 1;
    axle_i_is_in_group = [axle_i_is_in_group,counter*ones(1,Inputs.num_axles_per_group(group_num))];
end % for group_num = 1:num_groups
clear group_num counter

% ----------------- Symbolic variables definitions ------------------------
% More than necesary symbolic variables are generated below
% At a later stage, those unnecessary ones are deleted

% -- Bodies mechanical properties --
% mB = mass of body-i
% IBi = Pitch moment of inertia of body-i
B02_IndexedSymbVars('mB',num_bodies,'i');
B02_IndexedSymbVars('IB',num_bodies,'i');

% -- Suspension mechanical properties --
% kSj = Supension stiffness
% cSj = Suspension damping
B02_IndexedSymbVars('kS',num_groups,'j');
B02_IndexedSymbVars('cS',num_groups,'j');

% -- Axle/group mechanical properties --
% mGj = Axle/group mass
% IGj = Axle/group pitch moment of intertia
B02_IndexedSymbVars('mG',num_groups,'j');
B02_IndexedSymbVars('IG',num_groups,'j');

% -- Tyre mechanical properties --
% kTk = Tyre stiffness
% cTk = Tyre damping
B02_IndexedSymbVars('kT',num_axles,'k');
B02_IndexedSymbVars('cT',num_axles,'k');

% -- Distances from bodies centre of gravity with to articulations --
% ai = Horizontal distance from front articulation to centre of gravity of body-i
% bi = Horizontal distance from centre of gravity of body-i to back articulation
% Note that body-1 has no ai, and the last body has no bi
B02_IndexedSymbVars('a',num_bodies,'i');
B02_IndexedSymbVars('b',num_bodies-1,'i');

% -- Geometry of groups --
%dj = Horizontal coordinate from a group with respect to its corresponding body centre of gravity (Can have positive and negative values)
%ek = Horizontal coordinate from axle with respect to its axle group centre (Can have positive and negative values)
B02_IndexedSymbVars('d',num_groups,'j');
B02_IndexedSymbVars('e',num_axles,'k');

% % -- Bodies degrees of freedom --
% yBi = Vertical displacement of body-i centre of gravity
% thetaBi = Pitch angle of body-i centre of gravity
B02_IndexedSymbVars('yB',num_bodies,'i',0);
B02_IndexedSymbVars('yB',num_bodies,'i',1);
B02_IndexedSymbVars('yB',num_bodies,'i',2);
B02_IndexedSymbVars('thetaB',num_bodies,'i',0);
B02_IndexedSymbVars('thetaB',num_bodies,'i',1);
B02_IndexedSymbVars('thetaB',num_bodies,'i',2);

% % -- Axle groups degrees of freedom --
% yGj = Vertical displacement of mass of axle group i
% thetaGj = Rotation of the bar of axle group i
B02_IndexedSymbVars('yG',num_groups,'j',0);
B02_IndexedSymbVars('yG',num_groups,'j',1);
B02_IndexedSymbVars('yG',num_groups,'j',2);
B02_IndexedSymbVars('thetaG',num_groups,'j',0);
B02_IndexedSymbVars('thetaG',num_groups,'j',1);
B02_IndexedSymbVars('thetaG',num_groups,'j',2);

% -- Equations --
EQ = kS1*zeros(num_axles+num_bodies*2,2);     % Initialize matrix

% -- Road profile under each axle --
% rk = Vertical elevation of road profile for axle i
B02_IndexedSymbVars('r',num_axles,'k',0);
B02_IndexedSymbVars('r',num_axles,'k',1);

% -------------------- Processing of DOF ----------------------------------

% -- Making some DOFs equal to zero --
% Removing axle rotation of axle groups with only one axle
thetaGj = thetaGj.*(Inputs.num_axles_per_group>1);
thetaGj_d = thetaGj_d.*(Inputs.num_axles_per_group>1);
thetaGj_dd = thetaGj_dd.*(Inputs.num_axles_per_group>1);
% Removing body rotation if first (and only) body has only one supspension (Q-car)
if num_bodies == 1
    thetaBi = thetaBi.*([Body(:).num_axles]>1);
    thetaBi_d = thetaBi_d.*([Body(:).num_axles]>1);
    thetaBi_dd = thetaBi_dd.*([Body(:).num_axles]>1);
end % if num_bodies == 1

% -- DOF relations for articulation --
counter = 1;
for art_num = 1:num_articulations
    if Inputs.with_articulation(art_num) == 1
        % DOF relationship
        % yG = yT + aT*thetaT + aS*thetaG;
        DOF_relation{counter,1} = char(yBi(art_num+1));
        DOF_relation{counter,2} = char(yBi(art_num) + bi(art_num)*thetaBi(art_num) + ai(art_num+1)*thetaBi(art_num+1));
        eval([DOF_relation{counter,1},' = ',DOF_relation{counter,2},';']);
        counter = counter + 1;
        % First derivative expression
        DOF_d_relation = yBi_d(art_num) + bi(art_num)*thetaBi_d(art_num) + ai(art_num+1)*thetaBi_d(art_num+1);
        eval([char(yBi_d(art_num+1)),' = ',char(DOF_d_relation),';']);
        % Second derivative expression
        DOF_dd_relation = yBi_dd(art_num) + bi(art_num)*thetaBi_dd(art_num) + ai(art_num+1)*thetaBi_dd(art_num+1);
        eval([char(yBi_dd(art_num+1)),' = ',char(DOF_dd_relation),';']);
    end % if Inputs.with_articulation(art_num) == 1
end % for art_num = 1:num_articulations
clear art_num DOF_d_relation DOF_dd_relation counter

% -- Building vector of DOF --
% Bodies vertical displacements
DOF = yBi(1);
DOF_d = yBi_d(1);
DOF_dd = yBi_dd(1);
for art_num = 1:num_articulations
    if Inputs.with_articulation(art_num) == 0
        DOF = [DOF,yBi(art_num+1)];
        DOF_d = [DOF_d,yBi_d(art_num+1)];
        DOF_dd = [DOF_dd,yBi_dd(art_num+1)];
    end % if Inputs.with_articulation(art_num) == 0
end % for art_num = 1:num_articulations
% Bodies rotations
DOF = [DOF,thetaBi];
DOF_d = [DOF_d,thetaBi_d];
DOF_dd = [DOF_dd,thetaBi_dd];
% Axle(s) vertical displacement
DOF = [DOF,yGj];
DOF_d = [DOF_d,yGj_d];
DOF_dd = [DOF_dd,yGj_dd];
% Axle(s) rotation
DOF = [DOF,thetaGj];
DOF_d = [DOF_d,thetaGj_d];
DOF_dd = [DOF_dd,thetaGj_dd];
% All DOF
DOF_all = [DOF_dd,DOF_d,DOF];
clear DOF_d DOF_dd

% Removing zero entries in DOF_all
DOF_all(DOF_all==0) = [];
DOF(DOF==0) = [];

% Array to cell for later symbolic evaluation
for i = 1:length(DOF_all)
    DOF_all_cell{i} = DOF_all(i);
end % for i = 1:length(DOF_all)
for k = 1:length(rk)
    rk_and_rk_d_cell{k} = rk(k);
    rk_and_rk_d_cell{length(rk)+k} = rk_d(k);
end % for k = 1:length(rk)
clear i k

% -------------------- Auxiliary forces -----------------------------------

% Forces from each separate axle
F_from_axles = kS1*zeros(1,num_axles);
for axle_num = 1:num_axles
    group_num = axle_i_is_in_group(axle_num);
    F_from_axles(axle_num) = ...
        kTk(axle_num)*(rk(axle_num)-yGj(group_num)) + ...
        cTk(axle_num)*(rk_d(axle_num)-yGj_d(group_num));
    if Inputs.num_axles_per_group(group_num) > 1
        F_from_axles(axle_num) = F_from_axles(axle_num) + ...
            -kTk(axle_num)*thetaGj(group_num)*ek(axle_num) + ...
            -cTk(axle_num)*thetaGj_d(group_num)*ek(axle_num);
    end % if Inputs.num_axles_per_group(group_num) > 1
end % for axle_num = 1:num_axles
clear axle_num group_num

% Forces from each seaparate axle/group
F_from_suspension = kS1*zeros(1,num_groups);
for group_num = 1:num_groups
    body_num = group_i_is_in_body(group_num);
    F_from_suspension(group_num) = ...
        kSj(group_num)*(yGj(group_num)-yBi(body_num)-thetaBi(body_num)*dj(group_num)) + ...
        cSj(group_num)*(yGj_d(group_num)-yBi_d(body_num)-thetaBi_d(body_num)*dj(group_num));
end % for group_num = 1:num_groups
clear group_num body_num

% Internal forces Ri at the articulations (fifth wheels)
R = yBi*0;
for art_num = num_articulations:-1:1
    if Inputs.with_articulation(art_num) == 0
        R(art_num) = 0;
    elseif Inputs.with_articulation(art_num) == 1
        % Expression derived from unused Equation Type I
        R(art_num) = mBi(art_num+1)*yBi_dd(art_num+1) + R(art_num+1) ...
            - sum(F_from_suspension(Body(art_num+1).group_inds));
        R = expand(R);
    end % if Inputs.with_articulation(art_num) == 0
end % for art_num = 1:num_articulations
clear art_num

% Axle spacing and axle distance
ax_dist = kS1*0;
for axle_num = 1:num_axles
    group_num = axle_i_is_in_group(axle_num);
    body_num = group_i_is_in_body(group_num);
    if num_groups_with_theta == 0
        ax_dist(axle_num) = dj(group_num);
    else
        ax_dist(axle_num) = dj(group_num) + ek(axle_num);
    end % if num_groups_with_theta == 0
    if body_num > 1
        ax_dist(axle_num) = ax_dist(axle_num) + sum(bi(1:body_num-1)) + sum(ai(2:body_num));
    end % if body_num > 1
end % for axle_num = 1:num_axles
clear group_num body_num
ax_dist = ax_dist-ax_dist(1);
ax_sp = ax_dist(2:end)-ax_dist(1:end-1); ax_sp(2:end+1) = ax_sp; ax_sp(1) = ax_sp(1)*0;

% ---------------------- Equations of motion ------------------------------
% Each side of the n equation of motion is saved in separate vector cells
% So, equation number n is equal to: EQ(n,1) = EQ(n,2)

eq_num = 0;     % Equation counter

for body_num = 1:num_bodies
    if body_num == 1
        include_eq = 1;
    else
        if Inputs.with_articulation(body_num-1) == 0
            include_eq = 1;
        else
            include_eq = 0;
        end % if Inputs.with_articulation(body_num-1) == 0
    end % if body_num == 1
    if include_eq == 1
        % **** Equations Type I ****
        % Sum of Forces in Bodies (Each body in a separate equation)
        eq_num = eq_num + 1;
        EQ(eq_num,1) = mBi(body_num)*yBi_dd(body_num);
        for group_num = Body(body_num).group_inds
            EQ(eq_num,2) = EQ(eq_num,2) + F_from_suspension(group_num);
        end % for group_num = Body(body_num).group_inds
        % Addition of articulation forces
        if body_num == 1
            if Inputs.with_articulation(1) == 1
                EQ(eq_num,2) = EQ(eq_num,2) - R(1);
            end % if Inputs.with_articulation == 1
        elseif body_num == num_bodies
            if Inputs.with_articulation(end) == 1
                EQ(eq_num,2) = EQ(eq_num,2) + R(end-1);
            end % if Inputs.with_articulation == 1
        else
            if Inputs.with_articulation(body_num-1) == 1
                EQ(eq_num,2) = EQ(eq_num,2) + R(body_num-1);
            end % if Inputs.with_articulation == 1
            if Inputs.with_articulation(body_num) == 1
                EQ(eq_num,2) = EQ(eq_num,2) - R(body_num);
            end % if Inputs.with_articulation == 1
        end % if body_num == 1
    end % if include_eq == 1
end % for body_num = 1:num_bodies
clear body_num group_num include_eq

for body_num = 1:num_bodies
    % **** Equations Type II ****
    % Sum of Pitch moments in Bodies (Each body in a separate equation)
    if any([Body(body_num).num_axles > 1,...
            Art.Body(body_num).front == 1,...
            Art.Body(body_num).back == 1])
        eq_num = eq_num + 1;
        EQ(eq_num,1) = IBi(body_num)*thetaBi_dd(body_num);
        for group_num = Body(body_num).group_inds
            EQ(eq_num,2) = EQ(eq_num,2) + dj(group_num)*F_from_suspension(group_num);
        end % for group_num = Body(body_num).group_inds
        % Addition of moments due to articulation forces
        if body_num == 1
            if Inputs.with_articulation(1) == 1
                EQ(eq_num,2) = EQ(eq_num,2) - bi(1)*R(1);
            end % if Inputs.with_articulation == 1
        elseif body_num == num_bodies
            if Inputs.with_articulation(end) == 1
                EQ(eq_num,2) = EQ(eq_num,2) - ai(end)*R(end-1);
            end % if Inputs.with_articulation == 1
        else
            if Inputs.with_articulation(body_num-1) == 1
                EQ(eq_num,2) = EQ(eq_num,2) - ai(body_num)*R(body_num-1);
            end % if Inputs.with_articulation == 1
            if Inputs.with_articulation(body_num) == 1
                EQ(eq_num,2) = EQ(eq_num,2) - bi(body_num)*R(body_num);
            end % if Inputs.with_articulation == 1
        end % if body_num == 1
    end % if any
end % for body_num = 1:num_bodies
clear body_num group_num

% **** Rest of Equations **** For each axle group
% Sum of forces
for group_num = 1:num_groups
    eq_num = eq_num + 1;
    EQ(eq_num,1) = mGj(group_num)*yGj_dd(group_num);
    for axle_num = find(axle_i_is_in_group==group_num)
        EQ(eq_num,2) = EQ(eq_num,2) + F_from_axles(axle_num);
    end % for axle_num = find(axle_i_is_in_group==group_num)
    EQ(eq_num,2) = EQ(eq_num,2) - F_from_suspension(group_num);
end % for group_num = 1:num_tractor_groups

% Sum of moments
for group_num = 1:num_groups
    if Inputs.num_axles_per_group(group_num) > 1
        eq_num = eq_num + 1;
        EQ(eq_num,1) = IGj(group_num)*thetaGj_dd(group_num);
        for axle_num = find(axle_i_is_in_group==group_num)
            EQ(eq_num,2) = EQ(eq_num,2) + F_from_axles(axle_num)*ek(axle_num);
        end % for axle_num = find(axle_i_is_in_group==group_num)
    end % if Inputs.num_axles_per_group(group_num) > 1
end % for group_num = 1:num_groups

% Evaluating symbolic expressions (Effectively using DOF expressions)
EQ = eval(EQ);

% Removing extra EQ cells
EQ = EQ(1:eq_num,:);

% Making EQ = 0
EQ = EQ(:,1) - EQ(:,2);

% Expanding
EQ = expand(EQ);
num_DOF = eq_num;

fprintf('\b'); disp('  DONE');

% ------------------------ System Matrices --------------------------------

% Initialize Matrices
M = (EQ*0)*(EQ*0)'; C = M; K = M;
F = EQ*0;

% Builiding system matrices
disp('System Matrices generation ...');
for i = 1:eq_num
    disp([blanks(4),'Adding elements from Equation ',num2str(i)]);
    for j = 1:num_DOF*3
        aux1 = zeros(1,num_DOF*3);
        aux1(j) = 1;
        aux1 = subs(EQ(i),DOF_all_cell,{aux1});
        aux1 = subs(aux1,rk_and_rk_d_cell,{zeros(1,num_axles*2)});
        k = mod(j,num_DOF);
        if k == 0
            k = num_DOF; 
        end % if k == 0
        % Mass Matrix
        if (j/num_DOF) <= 1
            M(i,k) = M(i,k) + aux1;
        % Damping Matrix
        elseif and((j/num_DOF)>1,(j/num_DOF)<=2)
            C(i,k) = C(i,k) + aux1;
        % Stiffness Matrix
        else
            K(i,k) = K(i,k) + aux1;
        end % if (j/num_DOF) <= 1
    end % for j = 1:num_DOF*3
    
    % Force matrix
    aux1 = subs(EQ(i),DOF_all_cell,{zeros(1,num_DOF*3)});
    F(i) = - aux1;
end % for i = 1:eq_num
disp('System Matrices generation: DONE');

% Symmetry checks
disp('Checking system matrices:');
% Mass matrix
if sum(sum(abs(M-transpose(M)))) == 0
    disp([blanks(4),'M is symmetric = OK']);
else
    disp([blanks(4),'M is not symmetric = WRONG']);
end % if sum(sum(abs(M-transpose(M)))) == 0
% Damping matrix
if sum(sum(abs(C-transpose(C)))) == 0
    disp([blanks(4),'C is symmetric = OK']);
else
    disp([blanks(4),'C is not symmetric = WRONG']);
end % if sum(sum(abs(C-transpose(C)))) == 0
% Stiffness matrix
if sum(sum(abs(K-transpose(K)))) == 0
    disp([blanks(4),'K is symmetric = OK']);
else
    disp([blanks(4),'K is not symmetric = WRONG']);
end % if sum(sum(abs(K-transpose(K)))) == 0

% -------------------- Additional results ---------------------------------

disp('Generating additional results ...');

% Force vector for static response
DOF_with_grav = zeros(1,num_DOF);
for DOF_num = 1:num_DOF
    aux1 = char(DOF(DOF_num));
    DOF_with_grav(DOF_num) = aux1(1)=='y';
end % for DOF_num = 1:num_DOF
syms grav
F_static_no_grav = M*DOF_with_grav';

% Wheel contact force
% If we define the profile under each wheel and its derivatives as
%   rk = [r1 r2 r3 r4 ...];
%   rk_d = [r1_d r2_d r3_d r4_d ...];
% and corresponding wheel displacements
%   wk = [w1, w2, w3, w4, ...];
%   wk_d = [w1_d, w2_d, w3_d, w4_d, ...];
% Then the contact force of the vehicle on the floor is:
%   F_contact = Fc * (wk_d - rk_d) + Fk * (wk - rk)
Fk = (rk*0)'*(rk*0); Fc = Fk;   % Initialize
for i = 1:num_axles
    for j = 1:num_axles
        aux1 = zeros(1,num_axles);
        aux1(j) = 1;
        Fk(i,j) = subs(F(num_DOF-num_axles+i),rk_and_rk_d_cell,{[aux1,zeros(1,num_axles)]});
        Fc(i,j) = subs(F(num_DOF-num_axles+i),rk_and_rk_d_cell,{[zeros(1,num_axles),aux1]});
    end % for j = 1:num_axles
end % for i = 1:num_axles

% Nodal displacements to wheel displacements
N2w  = (ek*0)'*(DOF*0);
for wheel_num = 1:sum(Inputs.num_axles_per_group)
    for j = 1:num_DOF
        aux1 = zeros(1,num_DOF*3);
        aux1(num_DOF*2+j) = 1;
        N2w(wheel_num,j) = subs(yGj(axle_i_is_in_group(wheel_num)) + ...
            ek(wheel_num)*thetaGj(axle_i_is_in_group(wheel_num)),...
            DOF_all_cell,{aux1});
    end % for j = 1:num_DOF
end % for wheel_num = 1:sum(Inputs.num_axles_per_group)

fprintf('\b'); disp('  DONE');

% ---------------------- Results into function ----------------------------

% Saves into a m-file the system matrices generated

if Save.on == 1
    
    disp('Generating function file ...');

    % Checking existence and creating folder to save results
    aux1 = dir(Save.path_name);
    if isempty(aux1)
        mkdir(Save.path_name);
    end % if length(aux1) == 0
    clear aux1

    % Name of m-file
    scriptname = 'Vehicle';
    for body_num = 1:num_bodies
        scriptname = [scriptname,'_',num2str(Body(body_num).num_axles)];
    end % for body_num = 1:num_bodies
    if num_groups_with_theta > 0
        scriptname = [scriptname,'_G_',num2str(Inputs.num_axles_per_group)];
        scriptname = replace(scriptname,'  ','_');
    end % if num_groups_with_theta > 0
    if num_articulations > 0
        aux1 = find(scriptname=='_');
        aux1 = aux1(2:num_articulations+1).*Inputs.with_articulation;
        aux1(aux1==0) = [];
        scriptname(aux1) = 'A';
    end % if num_articulations > 0
    
    end % if num_articulations > 0

    % Aggregating function content by making a cell
    line = 1;
    
    % Function header
    myCell{line} = ['function [Veh] = ',scriptname,'(Veh)']; line = line + 1;
    myCell{line} = ['% ',scriptname]; line = line + 1;
    myCell{line} = ['% Created: ',datestr(now)]; line = line + 1;
    
    % One empty line
    myCell{line} = ' '; line = line + 1;
    
    % Vehicle information
    myCell{line} = '% -- Vehicle information --'; line = line + 1;
    myCell{line} = ['% num_axles_per_body = [',num2str(Inputs.num_axles_per_body),'];']; line = line + 1;
    myCell{line} = ['% num_axles_per_group = [',num2str(Inputs.num_axles_per_group),'];']; line = line + 1;
    myCell{line} = ['% with_articulation = [',num2str(Inputs.with_articulation),'];']; line = line + 1;
    
    % One empty line
    myCell{line} = ' '; line = line + 1;
    
    % Vehicle variables
    myCell{line} = '% -- Vehicle variables --'; line = line + 1;
    field_names = fields(Veh.Prop);
    for field_num = 1:length(field_names)
        try
            myCell{line} = ['% Veh.Prop.',field_names{field_num},' = [',...
                mychar(Veh.Prop.(field_names{field_num})(1,:)),']']; line = line + 1;
        end % try
    end % for field_num = 1:length(field_names)
    
    % One empty line
    myCell{line} = ' '; line = line + 1;
    
    % Degrees of Freedom
    myCell{line} = '% -- Degrees of Freedom --'; line = line + 1;
    for i = 1:length(DOF)
        myCell{line} = ['Veh.DOF(',num2str(i),').name = ''',mychar(DOF(i)),''';'];
        aux1 = char(DOF(i));
        if strcmp(aux1(1),'y')
            myCell{line} = [myCell{line},' Veh.DOF(',num2str(i),').type = ''displacement'';'];
        else
            myCell{line} = [myCell{line},' Veh.DOF(',num2str(i),').type = ''rotation'';'];
        end % if strcmp(aux1(1),'y')
        myCell{line} = [myCell{line},' Veh.DOF(',num2str(i),').dependency = ''independent'';'];
        line = line + 1;
    end % for i = 1:length(DOF)
    myCell{line} = ['Veh.DOF(1).num_independent = ',num2str(length(DOF)),';']; line = line + 1;
    clear i aux1;
    
    % One empty line
    myCell{line} = ' '; line = line + 1;
    
    % DOF relations
    myCell{line} = '% -- DOF relations -- '; line = line + 1;
    if exist('DOF_relation','var')
        DOF_extra = DOF;
        for i = 1:size(DOF_relation,1)
            % DOF name
            myCell{line} = ['Veh.DOF(',num2str(length(DOF)+i),').name = ''',DOF_relation{i,1},''';'];
            if strcmp(DOF_relation{i,1}(1),'y')
                myCell{line} = [myCell{line},' Veh.DOF(',num2str(length(DOF)+i),').type = ''displacement'';'];
            else
                myCell{line} = [myCell{line},' Veh.DOF(',num2str(length(DOF)+i),').type = ''dependent'';'];
            end % if strcmp(DOF_relation{i,1}(1),'y')
            myCell{line} = [myCell{line},' Veh.DOF(',num2str(length(DOF)+i),').dependency = ''dependent'';'];
            line = line + 1;
            % DOF relation (Expression)
            myCell{line} = ['Veh.DOF(',num2str(length(DOF)+i),').expression = ''',DOF_relation{i,1},' = ',DOF_relation{i,2},''';'];
            line = line + 1;
            % DOF relation (function)
            myCell{line} = ['Veh.DOF(',num2str(length(DOF)+i),').fun = @(y) ',DOF_relation{i,2},';'];
            for DOF_num = 1:length(DOF_extra)
                myCell{line} = replace(myCell{line},char(DOF_extra(DOF_num)),['y(',num2str(DOF_num),')']);
            end % for DOF_num = 1:length(DOF)
            DOF_extra = [DOF_extra, yBi(i+1)];
            field_names = fields(Veh.Prop);
            for field_num = 1:length(field_names)
                for ele_num = 1:length(Veh.Prop.(field_names{field_num}))
                    myCell{line} = replace(myCell{line},char(Veh.Prop.(field_names{field_num})(ele_num)),...
                        ['Veh.Prop.',field_names{field_num},'(',num2str(ele_num),')']);
                end % for ele_num = 1:length(Veh.Prop.(field_names{field_num}))
            end % for field_num = 1:length(field_names)
            line = line + 1;
        end % for i = size(DOF_relation,1)
        % Number of dependent DOF
        myCell{line} = ['Veh.DOF(1).num_dependent = ',num2str(size(DOF_relation,1)),';']; line = line + 1;
    else
        myCell{line} = 'Veh.DOF(1).num_dependent = 0;'; line = line + 1;
    end % if exist('DOF_relation','var')
    clear i DOF_num field_names field_num ele_num DOF_extra
    
    % One empty line
    myCell{line} = ' '; line = line + 1;
    
    % Vehicle axle spacing
    line_to_replace_from = line;
    myCell{line} = '% -- Axle spacing and distance --'; line = line + 1;
    myCell{line} = ['Veh.Prop.ax_sp = ',mychar(ax_sp),';']; line = line + 1;
    myCell{line} = ['Veh.Prop.ax_dist = ',mychar(ax_dist),';']; line = line + 1;
    
    % One empty line
    myCell{line} = ' '; line = line + 1;
    
    % Vehicle system matrices
    myCell{line} = '% -- Vehicle system matrices --'; line = line + 1;
    for matrix2print = ['M','C','K']
        % Matrix name
        myCell{line} = ['Veh.SysM.',matrix2print,' = ...']; line = line + 1;
        eval(['A = ',matrix2print,';']);
        % First row of matrix
        string2add = mychar(A(1,:));
        myCell{line} = [blanks(4),'[',string2add,'; ...']; line = line + 1;
        % Following rows of matrix
        for row_num = 2:size(A,1)-1
            string2add = mychar(A(row_num,:));
            myCell{line} = [blanks(4),string2add,'; ...']; line = line + 1;
        end % for row_num = 2:size(A,1)
        % Last row of matrix
        string2add = mychar(A(end,:));
        myCell{line} = [blanks(4),string2add,'];'];line = line + 1;
        % One empty line
        myCell{line} = ' '; line = line + 1;
    end % for matrix2print
    
    % Force vector for static response
    myCell{line} = '% -- Force vector to calculate static response --'; line = line + 1;
    myCell{line} = '% Note: When using this vector, multiply it by the gravity. Following the sign criteria defined here'; line = line + 1;
    myCell{line} = '%   gravity has negative value. The numerical value to use is grav = -9.81 m/s^2'; line = line + 1;
    myCell{line} = 'Veh.Static.F_vector_no_grav = [';
    for DOF_num = 1:num_DOF
        myCell{line} = [myCell{line},char(F_static_no_grav(DOF_num)),', '];
    end % for DOF_num = 1:num_DOF
    myCell{line} = [myCell{line}(1:end-2),'];']; line = line + 1;
    
    % One empty line
    myCell{line} = ' '; line = line + 1;
    
    % N2w
    myCell{line} = '% -- Nodal disp. to wheel disp. relation --'; line = line + 1;
    myCell{line} = 'Veh.SysM.N2w = ...'; line = line + 1;
    string2add = mychar(N2w(1,:));
    myCell{line} = [blanks(4),'[',string2add,'; ...']; line = line + 1;
    % Following rows of matrix
    for row_num = 2:size(N2w,1)-1
        string2add = mychar(N2w(row_num,:));
        myCell{line} = [blanks(4),string2add,'; ...']; line = line + 1;
    end % for row_num = 2:size(N2w,1)
    % Last row of matrix
    string2add = mychar(N2w(end,:));
    myCell{line} = [blanks(4),string2add,'];'];line = line + 1;
    
    % One empty line
    myCell{line} = ' '; line = line + 1;

    % Contact Force
    myCell{line} = '% -- Contact Force calculation matrices --'; line = line + 1;
    myCell{line} = 'Veh.Contact.expression_txt = ''F_contact = Fk*(wk-rk) + Fc*(wk_d-rk_d)'';'; line = line + 1;
    myCell{line} = '% where '; line = line + 1;
    myCell{line} = '%     wk = Vertical displacement of wheel'; line = line + 1;
    myCell{line} = '%     rk = Profile elevation'; line = line + 1;
    myCell{line} = '%     wk_d = First time derivative of vertical displacement of wheel'; line = line + 1;
    myCell{line} = '%     rk_d = First time derivative of profile elevation'; line = line + 1;
    myCell{line} = ' '; line = line + 1;
    for matrix2print = ['k','c']
        matrix2print = ['F',matrix2print];
        % Matrix name
        myCell{line} = ['Veh.Contact.',matrix2print,' = ...']; line = line + 1;
        eval(['A = ',matrix2print,';']);
        % First row of matrix
        string2add = mychar(A(1,:));
        myCell{line} = [blanks(4),'[',string2add,'; ...']; line = line + 1;
        % Following rows of matrix
        for row_num = 2:size(A,1)-1
            string2add = mychar(A(row_num,:));
            myCell{line} = [blanks(4),string2add,'; ...']; line = line + 1;
        end % for row_num = 2:size(A,1)
        % Last row of matrix
        string2add = mychar(A(end,:));
        myCell{line} = [blanks(4),string2add,'];'];line = line + 1;
        % One empty line
        myCell{line} = ' '; line = line + 1;
    end % for matrix2print = {'Fk','Fc'}

    % End of function
    myCell{line} = '% ---- End of function ----';

    % Adding brackets to relevant variables
    field_names = fields(Veh.Prop);
    for field_num = 1:length(field_names)
        counter = 0;
        for ele_num = 1:length(Veh.Prop.(field_names{field_num}))
            oldstring = char(Veh.Prop.(field_names{field_num})(ele_num));
            counter = counter + sum(count(myCell,oldstring));
            newstring = ['Veh.Prop.',field_names{field_num},'(',num2str(ele_num),')'];
            myCell = [{myCell{1:line_to_replace_from-1}},...
                replace({myCell{line_to_replace_from:end}},oldstring,newstring)];
        end % for ele_num = 1:length(Veh.Prop.(field_names{field_num}))
        % Removing unused variables
        if counter == length(Veh.Prop.(field_names{field_num}))
            myCell = {myCell{count(myCell,['% Veh.Prop.',field_names{field_num}])==0}};
            line_to_replace_from = line_to_replace_from - 1;
        end % if counter == 0
    end % for field_num = 1:length(field_names)
    clear field_names field_num counter ele_num oldstring newstring

    % Write cell into m-file
    fid = fopen([Save.path_name,scriptname,'.m'],'wt');
    for line = 1:numel(myCell)-1
        fprintf(fid,'%s\n',myCell{line});
    end % for line = 1:numel(myCell)-1
    fprintf(fid,'%s',myCell{line+1});
    fclose(fid);

    % Information display
    fprintf('\b'); disp('  DONE');
    disp(['Results saved in: ',scriptname,'.m']);

end % if Save.on == 1

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
function [input] = mychar(input)

% Transforms the input into a string/char
% Applies the appropriate command depending on the type of input
% Also removes unwanted expressions form char output, like "matrix"

if isnumeric(input)
    input = num2str(input);
else
    input = char(input);
    if length(input)>6
        if strcmp(input(1:6),'matrix')
            input = input(9:end-2);
        end % if strcmp
    end % if length(input)>6
end % if 
end % function

% ---- End of script ----