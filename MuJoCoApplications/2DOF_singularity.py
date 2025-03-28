import sys
import numpy as np
import mujoco
import mujoco_viewer

sys.path += [ "controllers", "utils" ]

from utils    import min_jerk_traj
from scipy.io import savemat

# Call the xml model file + data for MuJoCo
dir_name   = './models/my_robots/'
robot_name = '2DOF_planar_torque.xml'
model = mujoco.MjModel.from_xml_path( dir_name + robot_name )
data  = mujoco.MjData( model )

is_sing = True

# Create the viewer object
viewer = mujoco_viewer.MujocoViewer( model, data, hide_menus = True )

# Use the following type
# Type idx = 1: RRR
# Type idx = 2: RLR
idx = 2

# Set numpy print options
np.set_printoptions( precision = 4, threshold = 9, suppress = True )

# Parameters for the simulation
T        = 9.                       # Total Simulation Time
dt       = model.opt.timestep       # Time-step for the simulation (set in xml file)
fps      = 30                       # Frames per second
save_ps  = 1000                     # Saving point per second
n_frames = 0                        # The current frame of the simulation
n_saves  = 0                        # Update for the saving point
speed    = 1.0                      # The speed of the simulator

t_update = 1./fps     * speed       # Time for update 
t_save   = 1./save_ps * speed       # Time for saving the data

# The time-step defined in the xml file should be smaller than update
assert( dt <= t_update and dt <= t_save )

# The number of degrees of freedom
nq = model.nq

q1 = np.pi * 1/3
q_init = np.array( [ q1, np.pi-2*q1 ] )
data.qpos[ 0:nq ] = q_init
mujoco.mj_forward( model, data )

# The impedances of the robot 
Kp = 60 * np.eye( 3 )
Bp = 20 * np.eye( 3 )

# Save the references for the q and dq 
q  = data.qpos[ 0:nq ]
dq = data.qvel[ 0:nq ]

# Get the end-effector's ID
EE_site = "site_end_effector"
id_EE = model.site( EE_site ).id

# Saving the references 
p  = data.site_xpos[ id_EE ]
Jp = np.zeros( ( 3, model.nq ) )
Jr = np.zeros( ( 3, model.nq ) )

mujoco.mj_jacSite( model, data, Jp, Jr, id_EE )

dp = Jp @ dq

# Get the initial position of the robot's end-effector
# and also the other parameters
pi = np.copy( p )
pf = pi + np.array( [ 0.0, 2.0-pi[ 1 ], 0.0 ] )
t0 = 0.3
D  = 1.0

# Flags
is_save = True
is_view = True

# The data for mat save
t_mat   = [ ]
q_mat   = [ ] 
p_mat   = [ ] 
p0_mat  = [ ] 
dq_mat  = [ ] 
dp_mat  = [ ] 
dp0_mat = [ ] 
q0_mat  = [ ] 
Jp_mat  = [ ]

# Stable joint posture 
q0_L = np.array( [ 0.8 *np.pi, -0.6*np.pi ] )
q0_R = np.array( [ 0.2 *np.pi, np.pi-2*0.2*np.pi ] )

kq = 2

def is_singular_with_svd(matrix, threshold=1e-10):
    _, s, _ = np.linalg.svd(matrix)
    return np.min(s) < threshold

is_left  = False
is_right = False
gain = 0.0
gain_mat = []

stp = 0.005

while data.time <= T:

    mujoco.mj_step( model, data )
    p01, dp01, _ = min_jerk_traj( data.time, t0    , t0 + 1*D, pi, pf )
    p02, dp02, _ = min_jerk_traj( data.time, t0 + D, t0 + 2*D, np.zeros( 3 ), pi-pf )

    p03, dp03, _ = min_jerk_traj( data.time, t0 + 2*D, t0 + 3*D, np.zeros( 3 ), pf-pi )
    p04, dp04, _ = min_jerk_traj( data.time, t0 + 3*D, t0 + 4*D, np.zeros( 3 ), pi-pf )

    p05, dp05, _ = min_jerk_traj( data.time, t0 + 4*D, t0 + 5*D, np.zeros( 3 ), pf-pi )
    p06, dp06, _ = min_jerk_traj( data.time, t0 + 5*D, t0 + 6*D, np.zeros( 3 ), pi-pf )

    p0  =  p01 +  p02 +  p03 +  p04 +  p05 +  p06
    dp0 = dp01 + dp02 + dp03 + dp04 + dp05 + dp06

    # Torque 1: First-order Joint-space Impedance Controller
    mujoco.mj_jacSite( model, data, Jp, Jr, id_EE )

    dp = Jp @ dq

    tau_imp = Jp.T @ ( Kp @ ( p0 - p ) + Bp @ ( dp0 - dp ) )

    # If singularity management is on
    if is_sing:
        
        if data.time >= t0 + 1.0*D and data.time <= t0 + 2.0*D:
            q0 = q0_R
            gain = gain + stp if gain < 1.0 else 1.0

        elif data.time >= t0 + 2.0*D and data.time <= t0 + 3.0*D:
            gain = gain - stp if gain > 0.0 else 0.0

        elif data.time >= t0 + 3.0*D and data.time <= t0 + 4.0*D:
            if idx == 1:
                q0 = q0_R
            else:
                q0 = q0_L
            gain = gain + stp if gain < 1.0 else 1.0

        elif data.time >= t0 + 4.0*D and data.time <= t0 + 5.0*D:
            gain = gain - stp if gain > 0.0 else 0.0

        elif data.time >= t0 + 5.0*D and data.time <= t0 + 6.0*D:
            q0 = q0_R
            gain = gain + stp if gain < 1.0 else 1.0

        elif data.time >= t0 + 6.0*D:
            q0 = q0_R

        else:
            q0 = np.zeros( model.nq )
            gain = 0

        tau_sing = gain* ( kq * ( q0 - q ) )

    else:
        tau_sing = np.zeros( model.nq )


    # Adding the Torque
    data.ctrl[ : ] = tau_imp + tau_sing

    # Update Visualization
    if ( ( n_frames != ( data.time // t_update ) ) and is_view ):
        n_frames += 1
        viewer.render( )
        print( "[Time] %6.3f" % data.time )

    # Save Data
    if ( ( n_saves != ( data.time // t_save ) ) and is_save ):
        n_saves += 1

        t_mat.append(   np.copy( data.time ) )
        q_mat.append(   np.copy(  q  ) )
        dq_mat.append(  np.copy( dq  ) )
        p_mat.append(   np.copy(  p  ) )
        p0_mat.append(  np.copy( p0  ) )    
        dp_mat.append(  np.copy( dp  ) )
        dp0_mat.append( np.copy( dp0 ) )    
        Jp_mat.append(  np.copy(  Jp ) )
        gain_mat.append( gain )
        q0_mat.append( q0 )        

# Saving the data
if is_save:
    data_dic = { "t_arr": t_mat, "q_arr": q_mat, "p_arr": p_mat, "dp_arr": dp_mat, "q0_arr": q0_mat,
                 "p0_arr": p0_mat, "dq_arr": dq_mat, "dp0_arr": dp0_mat, "Kp": Kp, "Bq": Bp, "q0_L": q0_L, "q0_R":q0_R, "Jp_arr": Jp_mat, "Kq": kq, "gain":gain_mat }
    
    if idx == 1:
        savemat( "../data/MuJoCo/2DOF_sing_RRR.mat", data_dic )
    else:
        savemat( "../data/MuJoCo/2DOF_sing_RLR.mat", data_dic )

if is_view:            
    viewer.close()