%% build_simulink.m
%  Stewart Platform - Simscape Multibody Model Builder
%
%  Creates 'stewart_model.slx' programmatically.
%  The model uses Simscape Multibody and visualizes via Mechanics Explorer.
%
%  Requirements:
%    - Simscape
%    - Simscape Multibody
%
%  Usage:
%    >> build_simulink

fprintf('============================================\n');
fprintf('  Stewart Platform - Simulink Model Builder\n');
fprintf('============================================\n\n');

%% 1) Load parameters
if ~exist('L0', 'var') || ~exist('P_base', 'var')
    fprintf('[*] Running stewart_setup.m...\n');
    run('stewart_setup.m');
    fprintf('[ok] Parameters loaded.\n\n');
end

%% 2) Close and delete previous model if it exists
mdl = 'stewart_model';
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
if exist([mdl '.slx'], 'file')
    delete([mdl '.slx']);
end

%% 3) Create new empty model
fprintf('[*] Creating new model...\n');
new_system(mdl);
open_system(mdl);

set_param(mdl, ...
    'StopTime',   num2str(motion.T_sim), ...
    'Solver',     'ode15s', ...
    'SolverType', 'Variable-step');

%% 4) FOUNDATION BLOCKS
fprintf('[*] Adding foundation blocks...\n');

% Solver Configuration (required by Simscape)
try
    add_block('nesl_utility/Solver Configuration', [mdl '/Solver Config'], ...
        'Position', [20 20 150 60]);
catch
    add_block('fl_lib/Solver Configuration', [mdl '/Solver Config'], ...
        'Position', [20 20 150 60]);
end

% Mechanism Configuration (sets gravity direction)
add_block('sm_lib/Utilities/Mechanism Configuration', [mdl '/Mech Config'], ...
    'Position', [20 80 200 120]);

% World Frame
add_block('sm_lib/Frames and Transforms/World Frame', [mdl '/World Frame'], ...
    'Position', [20 150 120 190]);

%% 5) BASE PLATE (welded to world / ground)
fprintf('[*] Adding base plate...\n');

add_block('sm_lib/Body Elements/Solid', [mdl '/Base Plate'], ...
    'Position', [220 150 370 190]);

% Weld Joint fixes the base plate to ground
add_block('sm_lib/Joints/Weld Joint', [mdl '/Base Weld'], ...
    'Position', [140 150 215 190]);

add_line(mdl, 'World Frame/RConn1', 'Base Weld/LConn1', 'autorouting','on');
add_line(mdl, 'Base Weld/RConn1',   'Base Plate/LConn1','autorouting','on');

%% 6) TOP PLATE
fprintf('[*] Adding top plate...\n');

add_block('sm_lib/Body Elements/Solid', [mdl '/Top Plate'], ...
    'Position', [1050 150 1200 190]);

%% 7) SIX LEGS
%  Each leg topology:
%    Base Plate -> RigidTF_base -> SphericalJoint_base ->
%    LowerLegSolid -> PrismaticJoint -> UpperLegSolid ->
%    SphericalJoint_top -> RigidTF_top -> Top Plate
%
%  Actuation: From Workspace -> S-PS Converter -> Prismatic.q

fprintf('[*] Adding 6 legs (may take 1-2 minutes)...\n');

y0 = 250;   % vertical position of first leg [px]
dy = 150;   % vertical spacing between legs  [px]

for k = 1:6
    yk   = y0 + (k-1)*dy;
    kstr = num2str(k);
    fprintf('    Leg %d / 6...\n', k);

    % Block name strings
    n_tfb  = ['TF_Base_' kstr];
    n_sb   = ['SphB_'    kstr];
    n_ll   = ['LowerL_'  kstr];
    n_pris = ['Piston_'  kstr];
    n_ul   = ['UpperL_'  kstr];
    n_st   = ['SphT_'    kstr];
    n_tft  = ['TF_Top_'  kstr];
    n_src  = ['LegSrc_'  kstr];
    n_c2p  = ['S2PS_'    kstr];

    % --- Base Rigid Transform (offset to base attachment point) ---
    add_block('sm_lib/Frames and Transforms/Rigid Transform', [mdl '/' n_tfb], ...
        'Position', [390 yk 490 yk+40]);
    bp = P_base(k,:);
    try
        set_param([mdl '/' n_tfb], 'Translation', ...
            sprintf('[%g %g %g]', bp(1), bp(2), bp(3)));
    catch
        try
            set_param([mdl '/' n_tfb], 'R1Offset', ...
                sprintf('[%g %g %g]', bp(1), bp(2), bp(3)));
        catch
            warning('TF_Base_%d: Translation param not found. Manual fix may be needed.', k);
        end
    end

    % --- Base Spherical Joint (universal-like connection to ground) ---
    add_block('sm_lib/Joints/Spherical Joint', [mdl '/' n_sb], ...
        'Position', [505 yk 575 yk+40]);

    % --- Lower leg solid ---
    add_block('sm_lib/Body Elements/Solid', [mdl '/' n_ll], ...
        'Position', [590 yk 720 yk+40]);

    % --- Prismatic Joint (actuated piston motion) ---
    add_block('sm_lib/Joints/Prismatic Joint', [mdl '/' n_pris], ...
        'Position', [735 yk 820 yk+40]);
    try
        set_param([mdl '/' n_pris], 'PzMotionActuation', 'Provided by Input');
    catch
        try
            set_param([mdl '/' n_pris], 'PzActuatorType', 'Motion');
        catch
            warning('Piston_%d: Actuation mode not set. Configure manually after model update.', k);
        end
    end

    % --- Upper leg solid ---
    add_block('sm_lib/Body Elements/Solid', [mdl '/' n_ul], ...
        'Position', [835 yk 965 yk+40]);

    % --- Top Spherical Joint ---
    add_block('sm_lib/Joints/Spherical Joint', [mdl '/' n_st], ...
        'Position', [980 yk 1045 yk+40]);

    % --- Top Rigid Transform (offset to top attachment point, body frame) ---
    add_block('sm_lib/Frames and Transforms/Rigid Transform', [mdl '/' n_tft], ...
        'Position', [1060 yk 1160 yk+40]);
    tp = P_top_b(k,:);
    try
        set_param([mdl '/' n_tft], 'Translation', ...
            sprintf('[%g %g %g]', tp(1), tp(2), tp(3)));
    catch
        warning('TF_Top_%d: Translation param not found.', k);
    end

    % --- Actuator signal: From Workspace (delta from home) ---
    add_block('simulink/Sources/From Workspace', [mdl '/' n_src], ...
        'Position', [590 yk+60 720 yk+85]);

    leg_var_name = sprintf('leg_delta_%d', k);
    set_param([mdl '/' n_src], 'VariableName', leg_var_name, 'SampleTime', '-1');

    % Write delta data to base workspace
    delta_data = leg_lengths(:,k) - L0(k);
    leg_struct.time               = t_vec;
    leg_struct.signals.values     = delta_data;
    leg_struct.signals.dimensions = 1;
    assignin('base', leg_var_name, leg_struct);

    % --- Simulink -> Physical Signal Converter ---
    add_block('nesl_utility/Simulink-PS Converter', [mdl '/' n_c2p], ...
        'Position', [735 yk+60 820 yk+85]);

    % --- Connections within this leg ---
    try
        add_line(mdl, ['Base Plate/RConn' kstr], [n_tfb '/LConn1'], 'autorouting','on');
    catch
        try
            add_line(mdl, 'Base Plate/RConn1', [n_tfb '/LConn1'], 'autorouting','on');
        catch
        end
    end
    add_line(mdl, [n_tfb  '/RConn1'], [n_sb   '/LConn1'], 'autorouting','on');
    add_line(mdl, [n_sb   '/RConn1'], [n_ll   '/LConn1'], 'autorouting','on');
    add_line(mdl, [n_ll   '/RConn1'], [n_pris '/LConn1'], 'autorouting','on');
    add_line(mdl, [n_pris '/RConn1'], [n_ul   '/LConn1'], 'autorouting','on');
    add_line(mdl, [n_ul   '/RConn1'], [n_st   '/LConn1'], 'autorouting','on');
    add_line(mdl, [n_st   '/RConn1'], [n_tft  '/LConn1'], 'autorouting','on');

    try
        add_line(mdl, [n_tft '/RConn1'], ['Top Plate/LConn' kstr], 'autorouting','on');
    catch
        try
            add_line(mdl, [n_tft '/RConn1'], 'Top Plate/LConn1', 'autorouting','on');
        catch
        end
    end

    % Actuator signal chain
    add_line(mdl, [n_src '/1'],      [n_c2p '/1'],      'autorouting','on');
    try
        add_line(mdl, [n_c2p '/RConn1'], [n_pris '/q'], 'autorouting','on');
    catch
        try
            add_line(mdl, [n_c2p '/RConn1'], [n_pris '/Pz'], 'autorouting','on');
        catch
            warning('Piston_%d: Actuator port connection failed. Connect manually.', k);
        end
    end
end

fprintf('[ok] 6 legs added.\n');

%% 8) Save model
fprintf('[*] Saving model...\n');
save_system(mdl, [mdl '.slx']);
fprintf('[ok] stewart_model.slx saved.\n\n');

%% 9) Compile / update diagram
try
    fprintf('[*] Compiling diagram...\n');
    set_param(mdl, 'SimulationCommand', 'update');
    fprintf('[ok] Compilation successful. Ready to simulate.\n\n');
catch ME
    fprintf('[!] Compile error: %s\n', ME.message);
    fprintf('    Manual fixes may be needed (see warnings above).\n\n');
end

fprintf('============================================\n');
fprintf('To run the simulation:\n');
fprintf('  >> sim(''stewart_model'')\n');
fprintf('  OR press the Run button in Simulink.\n');
fprintf('Mechanics Explorer will open automatically.\n');
fprintf('============================================\n\n');
