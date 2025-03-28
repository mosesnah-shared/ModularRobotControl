%% Cleaning up + Environment Setup
clear; close all; clc;

% Change the matlab to current directory
cd(fileparts(matlab.desktop.editor.getActiveFilename));

% Add Explicit-MATLAB for visualization
% Activate setup Explicit-MATLAB
addpath( 'Explicit-MATLAB' );    
run('Explicit-MATLAB/setup.m');  % Replace 'myscript.m' with your actual script name

addpath( 'utils', 'DMPmodules', 'GeometryLibrary\MATLAB\' );    

% Configure default figure properties
fig_config( 'fontSize', 20, 'markerSize', 10 )

% Data directory
data_dir = '';

% Values, low
% Values Kp 600 Bp 40 Kr 70 Br 5 Kq 6 Bq 4.5 Tp 4
% Values Kp 600 Bp 40 Kr 70 Br 5 Kq 0 Bq 4.5 Tp 4

% Values, high
% Values Kp 800 Bp 80 Kr 70 Br 5 Kq 6 Bq 4.5 Tp 8

%% (1A) No joint stiffness
file_dir = '../data/KUKAresult/iiwa14_singularity_repeatability_Kq0_Tp8.txt';
fid = fopen( file_dir, 'r');

formatSpec = ['Time: %f  Joint Angle [ %f, %f, %f, %f, %f, %f, %f] ', ...
              'p0 Command[%f, %f, %f]'];

data = textscan(fid, formatSpec);
fclose(fid);

% Extract and reshape
t_arr   = data{1};                % [N x 1]
q_arr   = cell2mat(data(2:8))';   % [7 x N]
p0_arr  = cell2mat(data(9:11))';  % [3 x N]
Nt = length( t_arr );

%% (1B) Calculate the Forward Kinematics Map, plot the results

% Call the robot for Forward Kinematics
robot = iiwa14( 'high' );
robot.init( );

% Number of Sample Points 
Nt = length( t_arr );
R_raw = zeros( 3, 3, Nt );
p_raw = zeros( 3, Nt );

% Plot the Logarithm Map to check Repeatability 
for i = 1 : Nt
    H_tmp = robot.getForwardKinematics( q_arr( :, i ) );
    R_raw( :, :, i ) = H_tmp( 1:3, 1:3 );
    p_raw( :, i    ) = H_tmp( 1:3, 4 );
end

% starting index 
idx = 200;
idx1 = 533;

f = figure( ); a = axes( 'parent', f );
hold on; axis equal
plot3( p_raw( 1, idx:end),p_raw( 2, idx:end ),p_raw( 3, idx:end ), 'linewidth', 5, 'color', 'k')
plot3( p0_arr( 1, idx:idx+idx1 ),p0_arr( 2, idx:idx+idx1 ),p0_arr( 3, idx:idx+idx1 ), 'linewidth', 5, 'linestyle', ':', 'color', [0.6350 0.0780 0.1840] )
set( a, 'visible', 'off' )

exportgraphics( f, '../images/iiwa14_singularity_w_redundancy/no_Kq.pdf', 'ContentType', 'vector');
rmse = sqrt( mean((p_raw - p0_arr).^2, 2));


%% (2A) with joint stiffness
file_dir = '../data/KUKAresult/iiwa14_singularity_repeatability_Kq6_Tp8.txt';
fid = fopen( file_dir, 'r');

formatSpec = ['Time: %f  Joint Angle [ %f, %f, %f, %f, %f, %f, %f] ', ...
              'p0 Command[%f, %f, %f]'];

data = textscan(fid, formatSpec);
fclose(fid);

% Extract and reshape
t_arr   = data{1};                % [N x 1]
q_arr   = cell2mat(data(2:8))';   % [7 x N]
p0_arr  = cell2mat(data(9:11))';  % [3 x N]
Nt = length( t_arr );

%% (1B) Calculate the Forward Kinematics Map, plot the results

% Call the robot for Forward Kinematics
robot = iiwa14( 'high' );
robot.init( );

% Number of Sample Points 
Nt = length( t_arr );
R_raw = zeros( 3, 3, Nt );
p_raw = zeros( 3, Nt );

% Plot the Logarithm Map to check Repeatability 
for i = 1 : Nt
    H_tmp = robot.getForwardKinematics( q_arr( :, i ) );
    R_raw( :, :, i ) = H_tmp( 1:3, 1:3 );
    p_raw( :, i    ) = H_tmp( 1:3, 4 );
end

% starting index 
idx = 200;
idx1 = 533;

f = figure( ); a = axes( 'parent', f );
hold on; axis equal
plot3( p_raw( 1, idx:end),p_raw( 2, idx:end ),p_raw( 3, idx:end ), 'linewidth', 5, 'color', 'k' )
plot3( p0_arr( 1, idx:idx+idx1 ),p0_arr( 2, idx:idx+idx1 ),p0_arr( 3, idx:idx+idx1 ), 'linewidth', 5, 'linestyle', ':', 'color', [0.6350 0.0780 0.1840]	 )
set( a, 'visible', 'off' )

exportgraphics( f, '../images/iiwa14_singularity_w_redundancy/with_Kq.pdf', 'ContentType', 'vector');
rmse = sqrt( mean((p_raw - p0_arr).^2, 2));
