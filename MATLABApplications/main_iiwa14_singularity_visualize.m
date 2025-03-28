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

% Figure directory for saving:
fig_dir = '../images/iiwa14_singularity_visualize/';

%% (1-) No Joint-stiffness
%% (1A) Read the txt file 

file_dir = '../data/KUKAresult/iiwa14_singularity_repeatability_Kq0_Tp8.txt.txt';
fid = fopen( file_dir, 'r');

formatSpec = ['Time: %f  q values: [ %f, %f, %f, %f, %f, %f, %f] ', ...
              'p0 values: [%f, %f, %f]K gains%f'];

data = textscan(fid, formatSpec);
fclose(fid);

% Extract and reshape
t_arr   = data{1};                % [N x 1]
q_arr   = cell2mat(data(2:8))';   % [7 x N]
p0_arr  = cell2mat(data(9:11))';  % [3 x N]
k_gains = data{12};               % [N x 1]

%% (1B) Visualization

anim = Animation( 'Dimension', 3, 'xLim', [-0.3,1.1], 'yLim', [-0.7,0.7], 'zLim', [0,1.4] );
% Don't change the init order!
anim.init( );

idx_arr   = [300, 500, 700, 900, 1100, 1300, 1500];
Nr = length( idx_arr );

robots = cell( 1, Nr );

for i = 1 : Nr
    robot = iiwa14( 'high' );
    % Don't change the init order!
    robot.init( );
    robots{ i } = robot;
    anim.attachRobot( robot );
end

alpha_arr = 0.2*ones( 1, Nr );
alpha_arr( 1 ) = 1;

for i = 1 : Nr 

    robot = robots{ i };
    idx = idx_arr( i );
    % Using the whole robot workspace   
    alpha = alpha_arr( i );
    robot.updateKinematics( q_arr( :, idx ) )

    for j = 1 : 8
        anim.gPatches{ i }{ j }.FaceAlpha = alpha;
    end
end

anim.update( t_arr( idx ) )
set( anim.hAxes, 'visible', 'off' )

% Set view and then get pdf
set( anim.hAxes, 'view', [ 76.6271, -1.6386 ] )
exportgraphics( anim.hFig, [fig_dir, 'kin_sing_iiwa14_view1.jpeg']);

set( anim.hAxes, 'view', [ 136.1415, 14.0643 ] )
exportgraphics( anim.hFig, [fig_dir, 'kin_sing_iiwa14_view2.jpeg']);


%% (2A) Read the txt file 

% q virtual first values: [-0.056025,   0.80617,   0.30578,   -1.5212,  -0.08779,  -0.65851,         0]
% q virtual second values: [0.062657,   1.6788, -0.27908,   1.1467,  0.20281,  0.52674,  0.23876]

file_dir = '../data/KUKAresult/iiwa14_singularity_change_config.txt';
fid = fopen( file_dir, 'r');

formatSpec = ['Time: %f  q values: [ %f, %f, %f, %f, %f, %f, %f] ', ...
              'p0 values: [%f, %f, %f] Type: %d K gains%f'];

data = textscan(fid, formatSpec);
fclose(fid);

%% (2B) From up to down

q_up = [0.062657,   2.1788, -0.27908,   1.1467,  0.20281,  0.52674,  0.23876];

% Extract and reshape
t_arr   = data{1};               % [N x 1]
q_arr   = cell2mat(data(2:8))';  % [7 x N]
p0_arr  = cell2mat(data(9:11))'; % [3 x N]
k_gains = data{13};              % [N x 1]

anim = Animation( 'Dimension', 3, 'xLim', [-0.3,1.1], 'yLim', [-0.7,0.7], 'zLim', [0,1.4] );
% Don't change the init order!
anim.init( );

idx_arr   = [ 100, 180, 200, 370, 470 ];
Nr = length( idx_arr );

robots = cell( 1, Nr );

for i = 1 : Nr
    robot = iiwa14( 'high' );
    % Don't change the init order!
    robot.init( );
    robots{ i } = robot;
    anim.attachRobot( robot );
end

alpha_arr = cumsum( ones( 1, Nr )/Nr );

% Virtual configuration
virtual_robot = iiwa14( 'high' );
% Don't change the init order!
virtual_robot.init( );

% Modify the color 
for i = 2 : 8
    virtual_robot.gObjs.data{i}.f( :, 4 ) = 0.4940;
    virtual_robot.gObjs.data{i}.f( :, 5 ) = 0.1840;
    virtual_robot.gObjs.data{i}.f( :, 6 ) = 0.5560;
end

anim.attachRobot( virtual_robot )

for i = 1 : Nr 

    robot = robots{ i };
    idx = idx_arr( i );
    % Using the whole robot workspace   
    alpha = alpha_arr( i );
    robot.updateKinematics( q_arr( :, idx ) )

    for j = 1 : 8
        anim.gPatches{ i }{ j }.FaceAlpha = alpha;
    end
end

virtual_robot.updateKinematics( q_up )


for j = 1 : 8
    anim.gPatches{ Nr+1 }{ j }.FaceAlpha = 0.3;
end

anim.update( t_arr( idx ) )
set( anim.hAxes, 'visible', 'off' )
set( anim.hAxes, 'visible', 'off', 'view',[ 180.0, 0] )

exportgraphics( anim.hFig, [ fig_dir, '/kin_sing_iiwa14_up2down.jpeg' ]);


%% (2C) From down to up

q_down = [-0.056025,   0.80617,   0.30578,   -1.5212,  -0.08779,  -0.65851, 0];
% Extract and reshape
t_arr   = data{1};               % [N x 1]
q_arr   = cell2mat(data(2:8))';  % [7 x N]
p0_arr  = cell2mat(data(9:11))'; % [3 x N]
k_gains = data{13};              % [N x 1]

anim = Animation( 'Dimension', 3, 'xLim', [-0.3,1.1], 'yLim', [-0.7,0.7], 'zLim', [0,1.4] );
% Don't change the init order!
anim.init( );

idx_arr   = [ 470, 580, 650, 770, 800 ];
Nr = length( idx_arr );

robots = cell( 1, Nr );

for i = 1 : Nr
    robot = iiwa14( 'high' );
    % Don't change the init order!
    robot.init( );
    robots{ i } = robot;
    anim.attachRobot( robot );
end

alpha_arr = cumsum( ones( 1, Nr )/Nr );

% Virtual configuration
virtual_robot = iiwa14( 'high' );
% Don't change the init order!
virtual_robot.init( );

% Modify the color 
for i = 2 : 8
    virtual_robot.gObjs.data{i}.f( :, 4 ) = 0.4940;
    virtual_robot.gObjs.data{i}.f( :, 5 ) = 0.1840;
    virtual_robot.gObjs.data{i}.f( :, 6 ) = 0.5560;
end

anim.attachRobot( virtual_robot )

for i = 1 : Nr 

    robot = robots{ i };
    idx = idx_arr( i );
    % Using the whole robot workspace   
    alpha = alpha_arr( i );
    robot.updateKinematics( q_arr( :, idx ) )

    for j = 1 : 8
        anim.gPatches{ i }{ j }.FaceAlpha = alpha;
    end
end

virtual_robot.updateKinematics( q_down )

for j = 1 : 8
    anim.gPatches{ Nr+1 }{ j }.FaceAlpha = 0.3;
end

anim.update( t_arr( idx ) )
set( anim.hAxes, 'visible', 'off', 'view',[ 180.0, 0] )
exportgraphics( anim.hFig, [ fig_dir, '/kin_sing_iiwa14_up2down.jpeg' ]);