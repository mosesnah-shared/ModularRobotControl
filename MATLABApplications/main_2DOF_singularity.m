%% Cleaning up + Environment Setup
clear; close all; clc;

% Make sure that you add Explicit-MATLAB before running this script!
% Under "Explicit-MATLAB" directory, run setup.m file
cd(fileparts(matlab.desktop.editor.getActiveFilename));

% Configure default figure properties
fig_config( 'fontSize', 20, 'markerSize', 10 )

%% (Main) For 2DOF robot, R to L

close all
data = load( "../MuJoCoExamples/data/task_sing_RLR.mat" );

% Either 1 or 2
tmp_num = 2;
c_tmp = [0.6350 0.0780 0.1840];

mk = 2000;

if tmp_num == 1
    idx_arr = [ 900, 1450, 3900 ];
    step = 150;
else
    idx_arr = [ 4900, 5350, 5800 ];
    step = 100;
end

[ N, ~ ] = size( data.q_arr );

% The Robot Cartesian Coordinates
q_abs  = cumsum( data.q_arr, 2 );
x_arr = [ zeros( N, 1 ), cumsum( cos( q_abs ), 2 ) ];
y_arr = [ zeros( N, 1 ), cumsum( sin( q_abs ), 2 ) ];

cnt = 1;

% Elbow and End-effector
pEL = [ x_arr( :, 2 ), y_arr( :, 2 ) ];
pEE = [ x_arr( :, 3 ), y_arr( :, 3 ) ];

% The right and left configuration
qR = [ 0.2,  0.6 ]*pi;
qL = [ 0.8, -0.6 ]*pi;

% The right configuration
xR_arr = [ 0, cumsum( cos( cumsum( qR ) ) ) ];
yR_arr = [ 0, cumsum( sin( cumsum( qR ) ) ) ];

% The left  configuration
xL_arr = [ 0, cumsum( cos( cumsum( qL ) ) ) ];
yL_arr = [ 0, cumsum( sin( cumsum( qL ) ) ) ];

clr_act = [0.4940 0.1840 0.5560];



% Iterating through the indices
for idx = idx_arr
    
    % Getting the robot's end-effector, elbow and 
    f = figure( ); 
    a = subplot( 2, 2, [1,3], 'parent', f );
    hold on
    
    if cnt == 3
        step = 80;
    else
        step = 150;
    end

    % Left configuration
    Ntmp2 = 200;
    [X2D, Y2D] = meshgrid( linspace(-1.5, 1.5, Ntmp2 ), linspace(-0.4,2.6, Ntmp2 ) );
    U2 = zeros( Ntmp2, Ntmp2 );
    p0_init = data.p0_arr( idx, 1:2 );
    
    for i= 1:Ntmp2
        for j = 1:Ntmp2
            U2( i, j ) = 1/2*( [ X2D( i, j );Y2D( i, j )]' - p0_init ) * data.Kp(1:2,1:2) * ( [ X2D( i, j );Y2D( i, j )]' - p0_init )';
        end
    end
    
    U2 = U2.^(1/3);
%     surf( a , X2D,Y2D,zeros(size(U2)),U2,'edgecolor','none', 'facealpha', 0.5);
    colormap(flipud(jet))
    

    if ( cnt == 2 && tmp_num == 1 ) || ( cnt == 3 && tmp_num == 1 )
        qLf = plot( a, xL_arr, yL_arr, 'linewidth', 3, 'color', clr_act, 'linestyle', ':' );        
        scatter( a, xL_arr( 2 ), yL_arr( 2 ), 0.5*mk, 'o', 'filled', 'markerfacecolor', clr_act, 'linewidth', 4, 'markerfacealpha', 0.7 );
        scatter( a, xL_arr( 3 ), yL_arr( 3 ), 0.5*mk, 'o', 'filled', 'markerfacecolor', clr_act, 'linewidth', 4, 'markerfacealpha', 0.7 );
        qLf.Color( 4 ) = 0.4;
    end

    % Right configuration
    if ( cnt == 2 && tmp_num == 2 ) || ( cnt == 3 && tmp_num == 2 )
        qRf = plot( a, xR_arr, yR_arr, 'linewidth', 3, 'color', clr_act, 'linestyle', ':' );        
        scatter( a, xR_arr( 2 ), yR_arr( 2 ), 0.5*mk, 'o', 'filled', 'markerfacecolor', clr_act, 'linewidth', 4, 'markerfacealpha', 0.7 );
        scatter( a, xR_arr( 3 ), yR_arr( 3 ), 0.5*mk, 'o', 'filled', 'markerfacecolor', clr_act, 'linewidth', 4, 'markerfacealpha', 0.7 );
        qRf.Color( 4 ) = 0.4;
    end


    % To make it visually gradual
    ptmp1 = plot( a, x_arr( idx-1*step , : ), y_arr( idx-1*step, : ), 'linewidth', 3, 'color', 'k' );
    scatter( a, x_arr( idx-1*step , 2 ), y_arr( idx-1*step , 2 ), 0.2*mk, 'o', 'filled', 'markerfacecolor', 0.9*ones( 1, 3 ), 'markeredgecolor', 'k', 'linewidth', 4 , 'markerfacealpha', 1, 'markeredgealpha', 0.1 );
    scatter( a, x_arr( idx-1*step , 3 ), y_arr( idx-1*step , 3 ), 0.2*mk, 'o', 'filled', 'markerfacecolor', 0.9*ones( 1, 3 ), 'markeredgecolor', 'k', 'linewidth', 4 , 'markerfacealpha', 1, 'markeredgealpha', 0.1 );
    
    ptmp2 = plot( a, x_arr( idx-2*step , : ), y_arr( idx-2*step, : ), 'linewidth', 3, 'color', 'k' );
    scatter( a, x_arr( idx-2*step , 2 ), y_arr( idx-2*step , 2 ), 0.2*mk, 'o', 'filled', 'markerfacecolor', 0.7*ones( 1, 3 ), 'markeredgecolor', 'k', 'linewidth', 4 , 'markerfacealpha', 1, 'markeredgealpha', 0.1 );
    scatter( a, x_arr( idx-2*step , 3 ), y_arr( idx-2*step , 3 ), 0.2*mk, 'o', 'filled', 'markerfacecolor', 0.7*ones( 1, 3 ), 'markeredgecolor', 'k', 'linewidth', 4 , 'markerfacealpha', 1, 'markeredgealpha', 0.1 );
    
    ptmp1.Color( 4 ) = 0.1;
    ptmp2.Color( 4 ) = 0.1;

    % The robot Left configuration, and its markers
%     qLf = plot( a, xL_arr, yL_arr, 'linewidth', 3, 'color', 'k' );    
%     scatter( a, xL_arr( 2 ), yL_arr( 2 ), 0.3*mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 , 'markerfacealpha', 0.3);
%     scatter( a, xL_arr( 3 ), yL_arr( 3 ), 0.3*mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 , 'markerfacealpha', 0.3);
%     
%     % The robot Right configuration
%     qLf.Color( 4 ) = 0.3;

    % The robot current configuration
    plot( a, x_arr( idx, : ), y_arr( idx, : ), 'linewidth', 10, 'color', 'k' );
    
    % The markers for Shoulder (SH), Elbow (EL), and End-effector (EE)
    scatter( a, 0, 0, mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
    scatter( a, pEL( idx, 1 ), pEL( idx, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
    scatter( a, pEE( idx, 1 ), pEE( idx, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
    
    axis equal
    set( a, 'xlim', [-1.5, 1.5], 'ylim',[-0.4,2.6], 'xticklabel', {}, 'yticklabel', {})
    xlabel( a, '$X$ (m)', 'fontsize', 40 )
    ylabel( a, '$Y$ (m)', 'fontsize', 40 )
   
    % marker for p0
    scatter( a, data.p0_arr( idx, 1 ), data.p0_arr( idx, 2 ), 0.3*mk, 'o', 'filled', 'markerfacecolor', c_tmp, 'markeredgecolor', 'k', 'linewidth', 4 );
    
    % Plotting the torus
    a1 = subplot( 2, 2, [2,4], 'parent', f  );
    hold on
    % Define the parameters
    R = 5; % Major radius
    r = 2; % Minor radius
    
    % Generate the meshgrid for theta and phi
    Ntmp = 200;
    [q1, q2] = meshgrid(linspace(0, 1*pi, Ntmp), linspace(0, 2*pi, Ntmp));
    
    % Parametric equations for the torus
    X = (R + r*cos(q1)).*cos(q2);
    Y = (R + r*cos(q1)).*sin(q2);
    Z = r * sin(q1);
    
    % Plot the torus
    h = surf(a1, X, Y, Z, 'facealpha', 0.9, 'edgealpha', 0.0 );
    set( a1, 'view', [ -180, 30], 'visible', 'off' )
    q_init = data.q_arr( idx,:);

    axis equal
    X1 = (R + r*cos( q_init( 1 ) ) ).*cos( q_init( 2 ) );
    Y1 = (R + r*cos( q_init( 1 ) ) ).*sin( q_init( 2 ) );
    Z1 = r * sin( q_init( 1 ) );
    
    scatter3( a1, X1, Y1, Z1, 1000, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
    
    % Singularities
    X1 = (R + r*cos( pi/2 ) ).*cos( 0 );
    Y1 = (R + r*cos( pi/2 ) ).*sin( 0 );
    Z1 = r * sin( pi/2 );
    scatter3( a1, X1, Y1, Z1, 1000, 'd', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
    
    if ( cnt == 2 && tmp_num == 2 ) || ( cnt == 3 && tmp_num == 2 )
        XR = (R + r*cos( qR( 1 ) ) ).*cos( qR( 2 ) );
        YR = (R + r*cos( qR( 1 ) ) ).*sin( qR( 2 ) );
        ZR =      r*sin( qR( 1 ) );           
        scatter3( a1, XR, YR, ZR, 1500, "pentagram", 'filled', 'markerfacecolor', clr_act, 'markeredgecolor', 'k', 'linewidth', 2 );
    end

    if ( cnt == 2 && tmp_num == 1 ) || ( cnt == 3 && tmp_num == 1 )
        XL = (R + r*cos( qL( 1 ) ) ).*cos( qL( 2 ) );
        YL = (R + r*cos( qL( 1 ) ) ).*sin( qL( 2 ) );
        ZL =      r*sin( qL( 1 ) );        
        scatter3( a1, XL, YL, ZL, 1500, "pentagram", 'filled', 'markerfacecolor', clr_act, 'markeredgecolor', 'k', 'linewidth', 2 );
    end



    % Calculate the potential energy 
    % Forward Kinematics Map 
    x_FK = cos( q1 ) + cos( q1 + q2 );
    y_FK = sin( q1 ) + sin( q1 + q2 );
    
    p0_init = data.p0_arr( idx, 1:2 );
    
    U = zeros( Ntmp, Ntmp );
    for i= 1:Ntmp
        for j = 1:Ntmp
            U( i, j ) =1/2*( [ x_FK( i, j );y_FK( i, j )]' - p0_init ) * data.Kp(1:2,1:2) * ( [ x_FK( i, j );y_FK( i, j )]' - p0_init )';
        end
    end
    
    U = U.^(1/3);
    set( h, 'CData', U, 'FaceColor', 'interp' );
    % colormap(jet); % Use the 'jet' colormap or choose any other\
    colormap(flipud(jet))
    
    axis equal; % Equal scaling for all axes
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
%     title('Two-Dimensional Projection of a Torus');

    if tmp_num == 1
        exportgraphics( f, ['../images/kin_sing_2DOF_R2L_', num2str( cnt ), '.jpeg' ], 'ContentType', 'vector');
    else
        exportgraphics( f, ['../images/kin_sing_2DOF_L2R_', num2str( cnt ), '.jpeg' ], 'ContentType', 'vector');
    end
    
    cnt = cnt + 1;
end


%% (BACKUP) (A) For Task-space Position, no redundancy, Singularity, Type 1
% Using the Torus Manifold
close all

% Load the data
data = load( "../MuJoCoExamples/data/task_sing.mat" );
c_tmp = [0.6350 0.0780 0.1840];

mk = 2000;

[ N, ~ ] = size( data.q_arr );
dt = data.t_arr( 2 ) -  data.t_arr( 1 );
t_arr = dt * (0:(N-1));
T = max( data.t_arr );

% The actual Robot Locations
q_abs  = cumsum( data.q_arr, 2 );
x_arr = cumsum( cos( q_abs ), 2 );
y_arr = cumsum( sin( q_abs ), 2 );

% Adding the zeros
x_arr = [ zeros( N, 1 ), x_arr ];
y_arr = [ zeros( N, 1 ), y_arr ];

% Getting the robot's end-effector, elbow and 
f = figure( ); 
a = subplot( 2, 2, [1,3] );
hold on

Ntmp2 = 200;
[X2D, Y2D] = meshgrid( linspace(-1.5, 1.5, Ntmp2 ), linspace(-0.4,2.6, Ntmp2 ) );
U2 = zeros( Ntmp2, Ntmp2 );
p0_init = data.p0_arr( 1, 1:2 );

for i= 1:Ntmp2
    for j = 1:Ntmp2
        U2( i, j ) = 1/2*( [ X2D( i, j );Y2D( i, j )]' - p0_init ) * data.Kp(1:2,1:2) * ( [ X2D( i, j );Y2D( i, j )]' - p0_init )';
    end
end
U2 = U2.^(1/3);
h1 = surf( a , X2D,Y2D,zeros(size(U2)),U2,'edgecolor','none', 'facealpha', 0.5);
colormap(flipud(jet))


plot( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), 'linewidth', 4, 'color', 'k', 'linestyle', '--' )
scatter( a, data.p0_arr( 1, 1 ), data.p0_arr( 1, 2 ), 0.3*mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
scatter( a, data.p0_arr( end, 1 ), data.p0_arr( end, 2 ), 0.3*mk, 'd', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );

grobot  = plot( a, x_arr( 1, : ), y_arr( 1, : ), 'linewidth', 10, 'color', 'k' );
% Elbow and End-effector
pEL = [ x_arr( :, 2 ), y_arr( :, 2 ) ];
pEE = [ x_arr( :, 3 ), y_arr( :, 3 ) ];

gSH = scatter( a, 0, 0, mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
gEL = scatter( a, pEL( 1, 1 ), pEL( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
gEE = scatter( a, pEE( 1, 1 ), pEE( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );

axis equal
set( a, 'xlim', [-1.5, 1.5], 'ylim',[-0.4,2.6], 'xticklabel', {}, 'yticklabel', {})
xlabel( a, '$X$ (m)', 'fontsize', 40 )
ylabel( a, '$Y$ (m)', 'fontsize', 40 )

alpha = 0.3;

% The desired end-effector position
gEE0 = scatter( a, data.p0_arr( 1, 1 ), data.p0_arr( 1, 2 ), 0.3*mk, 'o', 'filled', 'markerfacecolor', c_tmp, 'markeredgecolor', 'k', 'linewidth', 4 );

a1 = subplot( 2, 2, 4 );
hold on
% Define the parameters
R = 5; % Major radius
r = 2; % Minor radius

% Generate the meshgrid for theta and phi
Ntmp = 200;
[q1, q2] = meshgrid(linspace(0, 1*pi, Ntmp), linspace(0, 2*pi, Ntmp));

% Parametric equations for the torus
X = (R + r*cos(q1)).*cos(q2);
Y = (R + r*cos(q1)).*sin(q2);
Z = r * sin(q1);

% Plot the torus
h = surf(a1, X, Y, Z, 'facealpha', 0.9, 'edgealpha', 0.0 );
set( a1, 'view', [ -180, 30], 'visible', 'off' )
q_init = data.q_arr(1,:);
axis equal
X1 = (R + r*cos( q_init( 1 ) ) ).*cos(q_init(2));
Y1 = (R + r*cos( q_init( 1 ) ) ).*sin(q_init(2));
Z1 = r * sin(q_init(1));

p_mark = scatter3( a1, X1, Y1, Z1, 1000, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );

% Singularities
X1 = (R + r*cos( pi/2 ) ).*cos( 0 );
Y1 = (R + r*cos( pi/2 ) ).*sin( 0 );
Z1 = r * sin( pi/2 );
scatter3( a1, X1, Y1, Z1, 1000, 'd', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );

% Calculate the potential energy 
% Forward Kinematics Map 
x_FK = cos( q1 ) + cos( q1 + q2 );
y_FK = sin( q1 ) + sin( q1 + q2 );

p0_init = data.p0_arr( 1, 1:2 );

U = zeros( Ntmp, Ntmp );
scl = 10;
for i= 1:Ntmp
    for j = 1:Ntmp
        U( i, j ) =1/2*( [ x_FK( i, j );y_FK( i, j )]' - p0_init ) * data.Kp(1:2,1:2) * ( [ x_FK( i, j );y_FK( i, j )]' - p0_init )';
    end
end

U = U.^(1/3);
set( h, 'CData', U, 'FaceColor', 'interp' );
% colormap(jet); % Use the 'jet' colormap or choose any other\
colormap(flipud(jet))

axis equal; % Equal scaling for all axes
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Two-Dimensional Projection of a Torus');

a2 = subplot( 2, 2, 2 );
copyobj( a1.Children, a2 );
a2.XLim = a1.XLim;
a2.YLim = a1.YLim;
a2.Title.String = a1.Title.String;
set( a2, 'visible', 'off' )

%% (BACKUP) (B) For Task-space Position, no redundancy, Singularity, Type RRR

% There are multiple types of movements that were produced.
% Two types exist: RRR or RLR
data = load( "../MuJoCoExamples/data/task_sing_RRR.mat" );

c_tmp1 = [0.6350 0.0780 0.1840];
c_tmp2 = [0.0000 0.4470 0.7410];

data = load( dir );


mk = 2000;

[ N, ~ ] = size( data.q_arr );
dt = data.t_arr( 2 ) -  data.t_arr( 1 );
t_arr = dt * (0:(N-1));
T = max( data.t_arr );

% Getting the robot's end-effector, elbow and 
f = figure( ); 
a = subplot( 2, 2, [1,3] );
hold on

% The actual Robot Locations
q_abs  = cumsum( data.q_arr, 2 );
x_arr = cumsum( cos( q_abs ), 2 );
y_arr = cumsum( sin( q_abs ), 2 );

% Adding the zeros
x_arr = [ zeros( N, 1 ), x_arr ];
y_arr = [ zeros( N, 1 ), y_arr ];

% The virtual Robot position 
q0_abs  = cumsum( data.q0_arr, 2 );
x0_arr = cumsum( cos( q0_abs ), 2 );
y0_arr = cumsum( sin( q0_abs ), 2 );

x0_arr = [ zeros( N, 1 ), x0_arr ];
y0_arr = [ zeros( N, 1 ), y0_arr ];

g0robot = plot( a, x0_arr( 1, : ), y0_arr( 1, : ), 'linewidth', 5, 'color', c_tmp2 );
g0robot.Color( 4 ) = data.gain( 1 );

pEL0 = [ x0_arr( :, 2 ), y0_arr( :, 2 ) ];
pEE0 = [ x0_arr( :, 3 ), y0_arr( :, 3 ) ];
gEL0 = scatter( a, pEL0( 1, 1 ), pEL0( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', c_tmp2, 'linewidth', 4 );
gEE01 = scatter( a, pEE0( 1, 1 ), pEE0( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', c_tmp2, 'linewidth', 4 );

% Plotting the color map on the R2 space
Ntmp2 = 200;
[X2D, Y2D] = meshgrid( linspace(-1.5, 1.5, Ntmp2 ), linspace(-0.4,2.6, Ntmp2 ) );
U2 = zeros( Ntmp2, Ntmp2 );
p0_init = data.p0_arr( 1, 1:2 );

for i= 1:Ntmp2
    for j = 1:Ntmp2
        U2( i, j ) = 1/2*( [ X2D( i, j );Y2D( i, j )]' - p0_init ) * data.Kp(1:2,1:2) * ( [ X2D( i, j );Y2D( i, j )]' - p0_init )';
    end
end
U2 = U2.^(1/3);
h1 = surf( a , X2D,Y2D,zeros(size(U2)),U2,'edgecolor','none', 'facealpha', 0.5);
colormap(flipud(jet))

% Elbow and End-effector
pEL = [ x_arr( :, 2 ), y_arr( :, 2 ) ];
pEE = [ x_arr( :, 3 ), y_arr( :, 3 ) ];
grobot  = plot( a, x_arr( 1, : ), y_arr( 1, : ), 'linewidth', 10, 'color', 'k' );

gSH = scatter( a, 0, 0, mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
gEL = scatter( a, pEL( 1, 1 ), pEL( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
gEE = scatter( a, pEE( 1, 1 ), pEE( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );

axis equal
set( a, 'xlim', [-1.5, 1.5], 'ylim',[-0.4,2.6], 'xticklabel', {}, 'yticklabel', {})
xlabel( a, '$X$ (m)', 'fontsize', 40 )
ylabel( a, '$Y$ (m)', 'fontsize', 40 )

alpha = 0.3;

% The desired end-effector position
gEE0 = scatter( a, data.p0_arr( 1, 1 ), data.p0_arr( 1, 2 ), 0.3*mk, 'o', 'filled', 'markerfacecolor', c_tmp1, 'markeredgecolor', 'k', 'linewidth', 4 );

a1 = subplot( 2, 2, 4 );
hold on
% Define the parameters
R = 5; % Major radius
r = 2; % Minor radius

% Generate the meshgrid for theta and phi
Ntmp = 200;
[q1, q2] = meshgrid(linspace(0, 1*pi, Ntmp), linspace(-pi, pi, Ntmp));

% Parametric equations for the torus
X = (R + r*cos(q1)).*cos(q2);
Y = (R + r*cos(q1)).*sin(q2);
Z = r * sin(q1);

% Plot the torus
h = surf(a1, X, Y, Z, 'facealpha', 0.9, 'edgealpha', 0.0 );
set( a1, 'view', [ -180, 30], 'visible', 'off' )
q_init = data.q_arr(1,:);
axis equal
X1 = (R + r*cos( q_init( 1 ) ) ).*cos(q_init(2));
Y1 = (R + r*cos( q_init( 1 ) ) ).*sin(q_init(2));
Z1 = r * sin(q_init(1));

p_mark = scatter3( a1, X1, Y1, Z1, 1000, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );

% Singularities
X1 = (R + r*cos( pi/2 ) ).*cos( 0 );
Y1 = (R + r*cos( pi/2 ) ).*sin( 0 );
Z1 = r * sin( pi/2 );
scatter3( a1, X1, Y1, Z1, 1000, 'd', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );

% Calculate the potential energy 
% Forward Kinematics Map 
x_FK = cos( q1 ) + cos( q1 + q2 );
y_FK = sin( q1 ) + sin( q1 + q2 );

p0_init = data.p0_arr( 1, 1:2 );

U = zeros( Ntmp, Ntmp );
for i= 1:Ntmp
    for j = 1:Ntmp
        U( i, j ) =1/2*( [ x_FK( i, j );y_FK( i, j )]' - p0_init ) * data.Kp(1:2,1:2) * ( [ x_FK( i, j );y_FK( i, j )]' - p0_init )';
    end
end

% Adding joint potential functinos
U_joint = zeros( Ntmp, Ntmp );
for i= 1:Ntmp
    for j = 1:Ntmp
        U_joint( i, j ) =1/2*data.Kq*sum( ( [ q1( i, j ); q2( i, j ) ] - data.q0_arr( 1, : )' ).^2 );
    end
end
U_joint = U_joint * data.gain( 1 );

U = (U + U_joint).^(1/3);
set( h, 'CData', U, 'FaceColor', 'interp' );
% colormap(jet); % Use the 'jet' colormap or choose any other\
colormap(flipud(jet))

axis equal; % Equal scaling for all axes
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Two-Dimensional Projection of a Torus');

a2 = subplot( 2, 2, 2 );
copyobj( a1.Children, a2 );
a2.XLim = a1.XLim;
a2.YLim = a1.YLim;
a2.Title.String = a1.Title.String;
set( a2, 'visible', 'off' )
