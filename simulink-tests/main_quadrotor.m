%% Quadrotor Plant Model - Main Entry Point
% Run this script to set up the quadrotor simulation environment
%
% Available scripts:
%   1. quadrotor_params.m      - Load physical parameters
%   2. quadrotor_dynamics.m    - Nonlinear dynamics function
%   3. create_quadrotor_simulink.m - Generate Simulink model
%   4. simulate_quadrotor.m    - Open-loop simulation
%   5. simulate_controlled.m   - Closed-loop PID simulation
%   6. quadrotor_linearize.m   - Linearization and LQR design
%   7. animate_quadrotor.m     - 3D visualization

%% Clear workspace
clear; clc; close all;

%% Set path
script_path = fileparts(mfilename('fullpath'));
cd(script_path);
addpath(script_path);

fprintf('========================================\n');
fprintf('   QUADROTOR PLANT MODEL SIMULATION\n');
fprintf('========================================\n\n');

%% Load parameters
fprintf('Loading quadrotor parameters...\n');
run('quadrotor_params.m');
fprintf('\n');

%% Menu
fprintf('Available simulations:\n');
fprintf('  [1] Open-loop simulation (hover test)\n');
fprintf('  [2] Closed-loop PID simulation (waypoint tracking)\n');
fprintf('  [3] Linearize and design LQR controller\n');
fprintf('  [4] Create Simulink model\n');
fprintf('  [5] Exit\n\n');

choice = input('Select option (1-5): ');

switch choice
    case 1
        fprintf('\nRunning open-loop simulation...\n');
        simulate_quadrotor;

    case 2
        fprintf('\nRunning closed-loop simulation...\n');
        simulate_controlled;

    case 3
        fprintf('\nLinearizing system...\n');
        quadrotor_linearize;

    case 4
        fprintf('\nCreating Simulink model...\n');
        create_quadrotor_simulink;

    case 5
        fprintf('\nExiting.\n');
        return;

    otherwise
        fprintf('\nInvalid choice. Running default (open-loop simulation).\n');
        simulate_quadrotor;
end

fprintf('\n========================================\n');
fprintf('Simulation complete!\n');
fprintf('========================================\n');
