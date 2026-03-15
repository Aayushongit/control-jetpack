%% Create Quadrotor Simulink Model
% This script programmatically creates a complete Simulink model
% for quadrotor dynamics simulation
%
% Run this script in MATLAB to generate 'quadrotor_plant.slx'

%% Clear workspace and close figures
clear; clc; close all;

%% Load parameters
run('quadrotor_params.m');

%% Model Configuration
modelName = 'quadrotor_plant';

%% Close existing model if open
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if exist([modelName '.slx'], 'file')
    delete([modelName '.slx']);
end

%% Create new Simulink model
new_system(modelName);
open_system(modelName);

%% Configure model settings
set_param(modelName, ...
    'StopTime', '30', ...
    'Solver', 'ode45', ...
    'MaxStep', '0.01', ...
    'RelTol', '1e-6', ...
    'AbsTol', '1e-8', ...
    'SaveTime', 'on', ...
    'SaveOutput', 'on');

%% ========== CREATE MODEL BLOCKS ==========

%% --- Input Subsystem ---
add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Motor_Inputs'], ...
    'Position', [50, 150, 150, 250]);

%% --- Force/Moment Calculation ---
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [modelName '/Mixing_Matrix'], 'Position', [200, 170, 300, 230]);

%% --- Quadrotor Dynamics Block ---
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [modelName '/Quad_Dynamics'], 'Position', [350, 140, 500, 260]);

%% --- State Integrator ---
add_block('simulink/Continuous/Integrator', [modelName '/Integrator'], ...
    'Position', [550, 185, 590, 225], ...
    'InitialCondition', '[0;0;0; 0;0;0; 0;0;0; 0;0;0]');

%% --- State Demultiplexer ---
add_block('simulink/Signal Routing/Demux', [modelName '/State_Demux'], ...
    'Position', [640, 150, 645, 260], 'Outputs', '12');

%% --- Output Subsystem for Position ---
add_block('simulink/Signal Routing/Mux', [modelName '/Position_Mux'], ...
    'Position', [700, 140, 705, 180], 'Inputs', '3');

%% --- Output Subsystem for Attitude ---
add_block('simulink/Signal Routing/Mux', [modelName '/Attitude_Mux'], ...
    'Position', [700, 190, 705, 230], 'Inputs', '3');

%% --- Output Subsystem for Velocity ---
add_block('simulink/Signal Routing/Mux', [modelName '/Velocity_Mux'], ...
    'Position', [700, 240, 705, 280], 'Inputs', '3');

%% --- Output Subsystem for Angular Velocity ---
add_block('simulink/Signal Routing/Mux', [modelName '/AngVel_Mux'], ...
    'Position', [700, 290, 705, 330], 'Inputs', '3');

%% --- Output Ports ---
add_block('simulink/Sinks/Out1', [modelName '/Position'], 'Position', [780, 155, 810, 175]);
add_block('simulink/Sinks/Out1', [modelName '/Attitude'], 'Position', [780, 205, 810, 225]);
add_block('simulink/Sinks/Out1', [modelName '/Velocity'], 'Position', [780, 255, 810, 275]);
add_block('simulink/Sinks/Out1', [modelName '/AngularVel'], 'Position', [780, 305, 810, 325]);

%% --- Scopes ---
add_block('simulink/Sinks/Scope', [modelName '/Position_Scope'], ...
    'Position', [850, 150, 880, 180]);
add_block('simulink/Sinks/Scope', [modelName '/Attitude_Scope'], ...
    'Position', [850, 200, 880, 230]);

%% --- Feedback Path ---
add_block('simulink/Signal Routing/Goto', [modelName '/State_Goto'], ...
    'Position', [750, 340, 810, 360], 'GotoTag', 'STATE');
add_block('simulink/Signal Routing/From', [modelName '/State_From'], ...
    'Position', [280, 270, 340, 290], 'GotoTag', 'STATE');

%% --- Constants for Parameters ---
add_block('simulink/Sources/Constant', [modelName '/Mass'], ...
    'Position', [280, 310, 320, 330], 'Value', 'quad.m');
add_block('simulink/Sources/Constant', [modelName '/Gravity'], ...
    'Position', [280, 340, 320, 360], 'Value', 'quad.g');

%% --- To Workspace ---
add_block('simulink/Sinks/To Workspace', [modelName '/State_Log'], ...
    'Position', [750, 370, 820, 400], ...
    'VariableName', 'state_log', ...
    'SaveFormat', 'Timeseries');

%% Connect lines
% Connect integrator output to demux
add_line(modelName, 'Integrator/1', 'State_Demux/1', 'autorouting', 'on');

% Connect demux to muxes
add_line(modelName, 'State_Demux/1', 'Position_Mux/1', 'autorouting', 'on');
add_line(modelName, 'State_Demux/2', 'Position_Mux/2', 'autorouting', 'on');
add_line(modelName, 'State_Demux/3', 'Position_Mux/3', 'autorouting', 'on');

add_line(modelName, 'State_Demux/4', 'Attitude_Mux/1', 'autorouting', 'on');
add_line(modelName, 'State_Demux/5', 'Attitude_Mux/2', 'autorouting', 'on');
add_line(modelName, 'State_Demux/6', 'Attitude_Mux/3', 'autorouting', 'on');

add_line(modelName, 'State_Demux/7', 'Velocity_Mux/1', 'autorouting', 'on');
add_line(modelName, 'State_Demux/8', 'Velocity_Mux/2', 'autorouting', 'on');
add_line(modelName, 'State_Demux/9', 'Velocity_Mux/3', 'autorouting', 'on');

add_line(modelName, 'State_Demux/10', 'AngVel_Mux/1', 'autorouting', 'on');
add_line(modelName, 'State_Demux/11', 'AngVel_Mux/2', 'autorouting', 'on');
add_line(modelName, 'State_Demux/12', 'AngVel_Mux/3', 'autorouting', 'on');

% Connect muxes to outputs
add_line(modelName, 'Position_Mux/1', 'Position/1', 'autorouting', 'on');
add_line(modelName, 'Attitude_Mux/1', 'Attitude/1', 'autorouting', 'on');
add_line(modelName, 'Velocity_Mux/1', 'Velocity/1', 'autorouting', 'on');
add_line(modelName, 'AngVel_Mux/1', 'AngularVel/1', 'autorouting', 'on');

% Connect to scopes
add_line(modelName, 'Position_Mux/1', 'Position_Scope/1', 'autorouting', 'on');
add_line(modelName, 'Attitude_Mux/1', 'Attitude_Scope/1', 'autorouting', 'on');

%% Save the model
save_system(modelName);

%% Display completion message
fprintf('\n');
fprintf('========================================\n');
fprintf('Simulink Model Created Successfully!\n');
fprintf('========================================\n');
fprintf('Model: %s.slx\n', modelName);
fprintf('\n');
fprintf('IMPORTANT: After running this script:\n');
fprintf('1. Double-click "Mixing_Matrix" block\n');
fprintf('2. Paste the mixing matrix code\n');
fprintf('3. Double-click "Quad_Dynamics" block\n');
fprintf('4. Paste the dynamics code\n');
fprintf('5. Connect remaining signal lines\n');
fprintf('========================================\n');

%% Open the model
open_system(modelName);
