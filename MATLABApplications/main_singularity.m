%% Cleaning up + Environment Setup
clear; close all; clc;

% Make sure that you add Explicit-MATLAB before running this script!

% Configure default figure properties
fig_config( 'fontSize', 20, 'markerSize', 10 )
%% (1A) Calculation of the Singular Points
robot = iiwa14( 'high' );
robot.init( );

N = 30;

% Define the total number of iterations
totalIter = N^5;
currIter  = 0;

% Initialize the progress bar
hBar = waitbar(0, 'Initializing...');

% Start the timer to track elapsed time
startT = tic;

% Iterate through the configurations, takes N^5 times of iterations
for a = 1:N 
    for b = 1:N
        for c = 1:N
            for d = 1:N
                for e = 1:N
                    % Increment the iteration counter
                    currIter = currIter + 1;

                    % Retrieve joint values from ranges
                    q2 = q2_range( a );
                    q3 = q3_range( b );
                    q4 = q4_range( c );
                    q5 = q5_range( d );
                    q6 = q6_range( e );

                    % Formulate the joint configuration array
                    q_arr = [ 0.0, q2, q3, q4, q5, q6, 0.0 ];
                    
                    % Compute Jacobian, mass matrix, and lambda_inv
                    JS = robot.getSpatialJacobian( q_arr );
                    M  = robot.getMassMatrix( q_arr );
                    lamInv = JS * inv( M ) * JS';
                
                    % Store the minimum singular value of lamInv
                    data_raw( a, b, c, d, e ) = min( svd( lamInv ) );

                    % Update the progress bar
                    progress   = currIter / totalIter;
                    elapsedT   = toc( startT );  % Get elapsed time in seconds
                    estTotalT  = elapsedT / progress;
                    estRemainT = estTotalT - elapsedT;
                    
                    % Convert estimated remaining time to hours, minutes, and seconds
                    hrs  = floor( estRemainT / 3600 );
                    mins = floor( mod( estRemainT, 3600 ) / 60);
                    secs = mod( estRemainT, 60 );

                    % Update the waitbar message
                    waitbar( progress, hBar, sprintf('Progress: %.2f%%, Time Remaining: %02d:%02d:%02.0f (hh:mm:ss)', ...
                        progress * 100, hrs, mins, secs));
                end
            end
        end
    end
end

% Close the progress bar when done
close(hBar);


%% (2A) Plotting the result

robot = iiwa14( 'high' );
robot.init( );

% Data is saved as data_raw
load( 'MATLABdata/iiwa14_sing_data_N30.mat' );

% Given a threshold value, take off the values that are smaller than thres
thres = 0.03;

% Raw indices
idx_arr = find( data_raw < thres );

% Taking off the values
data_thres = data_raw( idx_arr );

% Given an index, get the subscripts for plotting
[ d1, d2, d3, d4, d5 ] = ind2sub( size( data_raw ), idx_arr );

% Get the percentage of the increase 
perc = length( idx_arr )/numel( data_raw ) * 100;

anim = Animation( 'Dimension', 3, 'xLim', [-0.7,0.7], 'yLim', [-0.7,0.7], 'zLim', [0,1.4] );
anim.init( );
anim.attachRobot( robot );

% Plotting the values 
% The number of values that satisfy the threshold
Ns = length( idx_arr );
N  = 30;
q2_range = linspace( robot.q_min( 2 ), robot.q_max( 2 ), N );
q3_range = linspace( robot.q_min( 3 ), robot.q_max( 3 ), N );
q4_range = linspace( robot.q_min( 4 ), robot.q_max( 4 ), N );
q5_range = linspace( robot.q_min( 5 ), robot.q_max( 5 ), N );
q6_range = linspace( robot.q_min( 6 ), robot.q_max( 6 ), N );

p_new_arr = zeros( 1, 3 );
cnt = 1;
for i = 1 : 1e+3:Ns
    q2 = q2_range( d1( i ) );
    q3 = q3_range( d2( i ) );
    q4 = q4_range( d3( i ) );
    q5 = q5_range( d4( i ) );
    q6 = q6_range( d5( i ) );
    
    tmp = robot.getForwardKinematics( [ 0.0, q2, q3, q4, q5, q6, 0.0 ] );
    if i == 1
        p_old = tmp( 1:3, 4 );
        scatter3( anim.hAxes, p_old( 1 ), p_old( 2 ), p_old( 3 ), 10, 'linewidth', 5,'markeredgecolor', 'none', 'markerfacecolor', 'k','markerfacealpha', 0.1 )
        p_new_arr( cnt, : ) = p_old';
        cnt = cnt + 1;
    else
        p_new = tmp( 1:3, 4 ); 

        if( norm( p_old - p_new ) <= 0.1 )
            continue
        end
        p_old = p_new;
        p_new_arr( cnt, : ) = p_old';
        cnt = cnt + 1;
    end
    
    disp( i/Ns * 100 );
end

% Check distances and remove points within radius 0.5
Nt = length( p_new_arr );
keepIdx = true(Nt , 1); % Logical array to keep valid points
for i = 1:Nt 
    if keepIdx(i)
        distances = sqrt( sum( p_new_arr(i, :).^2 ));
        if distances < 0.7 || p_new_arr( i, 3 ) <= -0.01
            keepIdx( i ) = false; % Remove close points
        end
    end
end

% Compute Delaunay triangulation
% X = p_new_arr( :, 1 );
% Y = p_new_arr( :, 2 );
% Z = p_new_arr( :, 3 );

% Extract filtered points
X = p_new_arr( keepIdx, 1 );
Y = p_new_arr( keepIdx, 2 );
Z = p_new_arr( keepIdx, 3 );

DT = delaunayTriangulation( X, Y, Z );
% scatter3( anim.hAxes, X, Y, Z, 3, 'linewidth', 5,'markeredgecolor', 'k', 'markerfacecolor', 'w', 'markeredgealpha', 0.0, 'markerfacealpha', 0.1 )
% trisurf( DT.ConnectivityList, X, Y, Z, 'FaceAlpha', 0.03, 'EdgeColor', 'k', 'EdgeAlpha', 0.03, 'FaceColor', 'k');
set( anim.hAxes, 'xlim', [-1, 1], 'ylim', [-1, 1], 'zlim', [-0.4, 1.6])
set( anim.hAxes,'xticklabel', {},'yticklabel', {},'zticklabel', {})
box on
title( anim.hAxes, '' )
xlabel( anim.hAxes, '' )
ylabel( anim.hAxes, '' )
zlabel( anim.hAxes, '' )
view([0 1 0]); % Sets the view along X-axis

print( '../images/kin_sing_fig_wo_triangles.pdf', '-dpdf', '-bestfit')
