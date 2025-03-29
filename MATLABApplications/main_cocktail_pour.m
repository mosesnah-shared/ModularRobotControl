%% Cleaning up + Environment Setup
clear; close all; clc;

% Make sure that you add Explicit-MATLAB before running this script!
% Under "Explicit-MATLAB" directory, run setup.m file
cd(fileparts(matlab.desktop.editor.getActiveFilename));

% Add Explicit-MATLAB for visualization
% Activate setup Explicit-MATLAB
addpath( 'Explicit-MATLAB' );    
run('Explicit-MATLAB/setup.m');  % Replace 'myscript.m' with your actual script name

% Also, make sure you attach GeometryLibrary too
addpath( 'DMPmodules', 'utils', 'GeometryLibrary/MATLAB' )

% Configure default figure properties
fig_config( 'fontSize', 20, 'markerSize', 10 )


%% (1A) Read the txt file 

file_dir = '../KUKARobotApplications/iiwa14_cocktail_pour/data/pour1.txt';
fid = fopen( file_dir, 'r');

formatSpec = 'Time: %f  q values: [ %f, %f, %f, %f, %f, %f, %f] ';

data = textscan(fid, formatSpec);
fclose(fid);

% Extract and reshape
t_arr   = data{1}';                % [N x 1]
t_raw   = t_arr - t_arr( 1 );
q_raw   = cell2mat(data(2:8))';   % [7 x N]

%% (1A1) Calculate the Forward Kinematics Map 

% Call the robot for Forward Kinematics
robot = iiwa14( 'high' );
robot.init( );

% Number of Sample Points 
Nt = length( t_raw );
R_raw = zeros( 3, 3, Nt );
w_raw = zeros( 3, Nt );

% Plot the Logarithm Map to check Repeatability 
for i = 1 : Nt
    H_tmp = robot.getForwardKinematics( q_raw( :, i ) );
    R_raw( :, :, i ) = H_tmp( 1:3, 1:3 );
    w_raw( :, i )    = so3_to_R3( LogSO3( R_raw( :, :, i ) ) );
end

% Manually checking the start and end time to get to the next step
f = figure( ); a = axes( 'parent', f ); 
plot( a, t_raw, w_raw );
set( a, 'xlim', [ 0, max( t_raw ) ], 'fontsize', 25 );
xlabel( a, 'Time (s)', 'fontsize', 25 )

%% (1B) Calculate the Forward Kinematics Map 
% Trim the data for start and end, time manually discovered
T_trim1 = 0.4;
T_trim2 = 1.8;
Tmax = T_trim2 - T_trim1;

idx1 = find( min( abs( t_raw - T_trim1 ) ) == abs( t_raw - T_trim1 ) );
idx2 = find( min( abs( t_raw - T_trim2 ) ) == abs( t_raw - T_trim2 ) );

% Trim the data
t_demo = t_raw( idx1: idx2 ) - t_raw( idx1 );
q_demo = q_raw( :, idx1: idx2 );

% The number of sample points for the demo
P = length( t_demo );

% The joint velocity array from q_arr
% Using a simple numerical differentiation.
dq_demo = data_diff( q_demo, t_demo );


% Get the rotation matrix and angular velocity with respect to S
R_demo  = zeros( 3, 3, P );
dR_demo = zeros( 3, 3, P );
ws_demo = zeros( 3, P );

% The error vectors
 e_demo = zeros( 3, P );
de_demo = zeros( 3, P );

% Get the Forward Kinematic Map for the rotation matrix 
for i = 1:P
    H_demo = robot.getForwardKinematics( q_demo( :, i ) );
    J_demo = robot.getHybridJacobian( q_demo( :, i ) );
    
    R_demo( :, :, i )  = H_demo( 1:3, 1:3 );
    ws_demo( :, i )    = J_demo( 4:6, : ) * dq_demo( :, i );
    dR_demo( :, :, i ) = R3_to_so3( ws_demo( :, i ) ) * R_demo( :, :, i );
end

% Get the goal orientation, i.e., the final orientation
Rg = R_demo( :, :, end );

% Calculate the error vector and its derivative from Rg
for i = 1 : P
    R_tmp = R_demo( :, :, i )' * Rg;
    e_demo( :, i ) = so3_to_R3( LogSO3( R_tmp ) );

    % Theta, i.e., the angular difference
    tt = norm( e_demo( :, i ) );

    % The time derivative of R
    dR_tmp = dR_demo( :, :, i )' * Rg;
    
    % Refer to the Appendix of the Thesis
    if tt == 0 
        term1 = -1/8 * trace( dR_tmp ) * ( R_tmp - R_tmp' ); 
        term2 =  1/2 * ( dR_tmp - dR_tmp' );
    else
        term1 = ( tt*cos( tt ) - sin( tt ) )/( 4*sin( tt )^3 ) * trace( dR_tmp ) * ( R_tmp - R_tmp' );
        term2 = tt/( 2*sin( tt ) ) * ( dR_tmp - dR_tmp' );
    end

    de_demo( :, i ) = so3_to_R3( term1 - term2 );

end

% Smooth out the de_arr
de_demo_filt = smoothdata( de_demo, 2, "gaussian", 50 );

% Diff to get dde_arr and its filtered one
dde_demo = data_diff( de_demo_filt, t_demo );
dde_demo_filt = smoothdata( dde_demo, 2, "gaussian", 50 );

% Visualization of the whole result
f = figure( ); a = axes( 'parent', f );
a1 = subplot( 3, 2, [ 1, 2 ] ); hold on;
plot( a1, t_demo, e_demo )
set( a1, 'xlim', [ 0, Tmax ], 'xticklabel', {}, 'yticklabel', {} )
ylabel( a1, '$\mathbf{e}^{(d)}(t)$', 'fontsize', 50 )

a2 = subplot( 3, 2, 3 ); hold on;
plot( a2, t_demo, de_demo );
set( a2, 'xlim', [ 0, Tmax ], 'xticklabel', {}, 'yticklabel', {} )
ylabel( a2, '$\dot{\mathbf{e}}^{(d)}(t)$', 'fontsize', 50 )

a3 = subplot( 3, 2, 4 ); hold on;
plot( t_demo, de_demo_filt )
set( a3, 'xlim', [ 0, Tmax ], 'xticklabel', {}, 'yticklabel', {} )

a4 = subplot( 3, 2, 5 ); hold on;
plot( t_demo, dde_demo )
set( a4, 'xlim', [ 0, Tmax ], 'xticklabel', {}, 'yticklabel', {} )
ylabel( a4, '$\ddot{\mathbf{e}}^{(d)}(t)$', 'fontsize', 50 )

a5 = subplot( 3, 2, 6 ); hold on;
plot( t_demo, dde_demo_filt )
set( a5, 'xlim', [ 0, Tmax ], 'xticklabel', {}, 'yticklabel', {} )

exportgraphics( f, '../images/cocktail_pouring/imit_learning_pour1.pdf', 'ContentType', 'vector');

%% (1C) Conduct Imitation Learning and Rollout

as = 1.0;
az = 2000;
bz = 0.25*az;
T  = t_demo( P );
N  = 50;

% The three elements of DMP
cs        = CanonicalSystem( 'discrete', T, as );
trans_sys = TransformationSystem( az, bz, cs );
fs        = NonlinearForcingTerm( cs, N );

% Calculating the B matrix (or array)
B_mat = trans_sys.get_desired( e_demo, de_demo_filt, dde_demo_filt, zeros( 3, 1 ) );

% The phi matrix 
A_mat = zeros( N, P );

% Calculate the Phi matrix for Weight Learning
% Note that compared to Ijspeert 2013, this followed Koutras and Doulgeri (2020)
for i = 1 : P 
    t = t_demo( i );
    A_mat( :, i ) = fs.calc_multiple_ith( t, 1:N )/ fs.calc_whole_at_t( t ) * cs.calc( t );
end

% Learning the weights with Linear Least-square fitting
weight = B_mat * A_mat' * inv( A_mat * A_mat' );

% Actual rollout 
t0i   = 0.0;
dt    = 1e-3;
T     = 1.5;
t_arr = 0:dt:T;
Nt    = length( t_arr );

% The scaling factor 
scl = 1.0;

input_arr = fs.calc_forcing_term( t_arr( 1:end-1 ), weight, t0i, eye( 3 ) );
[ e_arr, ~, ~, ] = trans_sys.rollout( scl*e_demo( :, 1 ), zeros( 3, 1 ), zeros( 3, 1 ), scl*input_arr, t0i, t_arr ); 

% Plot the difference between 
f = figure( ); hold on;
plot( t_demo, e_demo, 'linewidth',  3, 'color', 'k' );
plot(  t_arr,  e_arr, 'linewidth', 10, 'color', 'k', 'linestyle', '--' );

R_arr = zeros( 3, 3, Nt );

% And also, use this as the delta movement. 

for i = 1 : Nt
    R_arr( :, :, i ) = Rg * ExpSO3( R3_to_so3( e_arr( :, i ) ) )';
end

R1 = R_arr( :, :, 1 );

for i = 1 : Nt
    R_arr( :, :, i ) = R1' * R_arr( :, :, i );
end

% Reshaping A into a 3x(3*N) array
R_arr_save = reshape( R_arr, 3, []);

% Saving the data as csv file
csv_filename = '../KUKARobotApplications/iiwa14_cocktail_pour/dataread/pour_1p0scl.csv';
writematrix( R_arr_save, csv_filename);

%% (1C2) Plot the R3 movement, on the SO(3) Manifold
Nt    = length( t_arr );

% The exponential coordinates 
e_arr_cmd = e_arr;

% Define the radius
r = pi;

% Generate the unit sphere data
[x, y, z] = sphere(50);  % 50 controls the resolution

% Scale the unit sphere to radius pi
x = r * x;
y = r * y;
z = r * z;

% Plot the sphere
f = figure( ); a = axes( 'parent', f );
hold on
s = surf(a, x, y, z);
axis equal;

% Make the surface transparent and remove colormap
set(s, 'FaceAlpha', 0.2, 'EdgeAlpha', 0.1, 'FaceColor', [0.5 0.5 0.5]);  % light gray

scl_arr = [ 0.3, 0.5, 0.7, 1.0, 1.2, 1.5, 2.0, 2.5 ];
Ns = length( scl_arr );

for i = 1 : Ns
    scl = scl_arr( i );
    scatter3( a, scl * e_arr_cmd( 1, 1 ), scl * e_arr_cmd( 2, 1 ), scl * e_arr_cmd( 3, 1 ), 100,'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 3)
    plot3( a, scl * e_arr_cmd( 1, : ), scl * e_arr_cmd( 2, : ), scl * e_arr_cmd( 3, : ), 'linewidth', 2, 'color', 'k' )
end

scatter3( a, 0, 0, 0, 100,'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 3)
scatter3( a, e_arr_cmd( 1, 1 ), e_arr_cmd( 2, 1 ), e_arr_cmd( 3, 1 ), 100,'filled', 'markerfacecolor', [0 0.4470 0.7410], 'markeredgecolor', 'k', 'linewidth', 3)
plot3( a, e_arr_cmd( 1, : ), e_arr_cmd( 2, : ), e_arr_cmd( 3, : ), 'linewidth', 4, 'color', [0 0.4470 0.7410]	 )

set( a, 'visible', 'off', 'view', [ 46.0050, 2.4334] )
exportgraphics( f, '../images/imit_learning_pour2.pdf', 'ContentType', 'vector');

%% (1C3) Draw the Frame plots

f = figure( ); a = axes( 'parent', f );
hold on; axis equal;

idx_arr = 100:100:1400;
Nidx = length( idx_arr );

% Parameters
scale = 1.0;  % length of axis arrows
offset_step = 1.5;  % how far apart each frame is along y-axis
lw = 6;

for i = 1:Nidx

    idx = idx_arr( i );
    R = R_arr( :, :, idx );  % 3x3 rotation matrix

    % Origin offset for this frame
    origin = [(i-1)*offset_step; 0;  0];

    % Each column of R is a direction vector for X (red), Y (green), Z (blue)
    x_axis = R(:,1);
    y_axis = R(:,2);
    z_axis = R(:,3);

    % Plot quiver3 for each axis
    quiver3(a, origin(1), origin(2), origin(3), ...
            scale*x_axis(1), scale*x_axis(2), scale*x_axis(3), ...
            'r', 'LineWidth', lw , 'MaxHeadSize', 1.0);
    quiver3(a, origin(1), origin(2), origin(3), ...
            scale*y_axis(1), scale*y_axis(2), scale*y_axis(3), ...
            'g', 'LineWidth', lw , 'MaxHeadSize', 1.0);
    quiver3(a, origin(1), origin(2), origin(3), ...
            scale*z_axis(1), scale*z_axis(2), scale*z_axis(3), ...
            'b', 'LineWidth', lw , 'MaxHeadSize', 1.0);

    % Plot origin marker
    plot3(a, origin(1), origin(2), origin(3), ...
          'o', 'MarkerSize', 6, ...
          'MarkerEdgeColor', 'k', ...
          'MarkerFaceColor', 'w');
end

set(a, 'xticklabel', {}, 'yticklabel', {}, 'zticklabel', {} )
set(a, 'xlim', [-offset_step, Nidx*offset_step], 'ylim', [-2,2], 'zlim', [-2,2], 'view', [20.6093, 20.2935])

exportgraphics( f, '../images/imit_learning_pour3.pdf', 'ContentType', 'vector');

%% (1D) Replay the movements 

anim = Animation( 'Dimension', 3, 'xLim', [-0.2,1.2], ...
                  'yLim', [-0.7,0.7], 'zLim', [0,1.4], 'isSaveVideo', false, 'VideoSpeed', 1.0 );
anim.init( );
anim.attachRobot( robot )  
anim.Title = 'KUKA_sim';

set( anim.hAxes, 'xticklabel', {}, 'yticklabel', {}, 'zticklabel', {} )
xlabel( anim.hAxes, '' )
ylabel( anim.hAxes, '' )
zlabel( anim.hAxes, '' )
set( anim.SubTitle, 'fontsize', 40 )

% Set the patch's transparency
% for i = 6 : robot.nq+1
%     anim.gPatches{ i }.FaceAlpha = 0.2;
% end

robot.q_init = [ -10.00 * pi/180;
                  46.19 * pi/180;
                  17.52 * pi/180;
                 -87.16 * pi/180;
                  -5.03 * pi/180;
                 -37.73 * pi/180;
                  0.000 * pi/180];

tmp = robot.getForwardKinematics( robot.q_init );
p0i = tmp( 1:3, 4 );
Ri  = tmp( 1:3, 1:3 );

% Conduct the delta movements
for i = 1 : Nt
    R_sim( :, :, i ) = Ri * R_arr( :, :, i );
end

robot.updateKinematics( robot.q_init );
anim.update( 0 );

% Time step for the simulation
ns = 1;

% Task-space impedances for Position
Kp = 400 * eye( 3 );
Bp = 0.1 * Kp;

% Joint-space impedance, damping
Bq = 1.6 * eye( robot.nq );
Kq = 0.0 * eye( robot.nq );

% Task-space impedances for Orientation
Kr = 5.0 * eye( 3 );
Br = 0.5 * eye( 3 );

% Initial joint posture and velocity
q  = robot.q_init;
dq = zeros( robot.nq, 1 );

% Title: simulation time
titleString = sprintf( 'Time: %2.1f sec', 0 );
mytitle = title( titleString );
set( mytitle, 'FontSize' , 15);

view( 90, 0)
% view( 120.5, 30 )

for i = 1:Nt

    t = t_arr( i );
    
    % Get the mass matrix of the Acrobot
    M = robot.getMassMatrix( q );
    C = robot.getCoriolisMatrix( q, dq );

    % Get the Hybrid Jacobian 
    JH = robot.getHybridJacobian( q );
    
    % Get the end-effector position and velocity 
    dp = JH( 1:3, : ) * dq;
    
    % The initial end-effector position 
    H = robot.getForwardKinematics( q );
    R = H( 1:3, 1:3 );
    p = H( 1:3,   4 );

    tau1 = JH( 1:3, : )' * ( Kp * ( p0i - p ) + Bp * ( - dp ) );
    tau2 = JH( 4:6, : )' * Kr * R * so3_to_R3( LogSO3( R' * R_sim( :, :, i ) ) );
    tau3 = - Bq * dq;
    tau  = tau1 + tau2 + tau3;

    rhs = M\( -C * dq + tau ); 

    [ q1, dq1 ] = func_symplecticEuler( q, dq, rhs, dt );
    q  =  q1;
    dq = dq1;
   
    if round( t / (anim.FrameUpdateTime) ) >= ns
        % Update the linkage plot
        robot.updateKinematics( q );
        anim.update( t );    
        ns = ns + 1;

        % Set animation title
        set( mytitle, 'String', sprintf( 'Time: %2.1f sec', t ) );     
    end
                                       
end

anim.close( )
