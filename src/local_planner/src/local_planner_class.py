#!/usr/bin/env python

#
# AER 1516 Motion Planning Project
# Local Planner
#

from __future__ import division, print_function, absolute_import
from cmath import pi
from enum import Enum

# Import libraries
import sys, copy, threading
import rospy
import numpy as np
import mosek as mosek
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis, quaternion_multiply

# Helper functions
from bezier_curves import bezier_curve, bezier_interpolate

# Import message types
from nav_msgs.msg import Path
from snapstack_msgs.msg import State, Goal
from geometry_msgs.msg import Point, PointStamped, Vector3, Quaternion, PoseStamped, Pose
from shape_msgs.msg import Plane
from std_msgs.msg import Float64, Header
from convex_decomposer.msg import CvxDecomp, Polyhedron
from master_node.msg import MasterNodeState

class LocalPlanner(object):
    "Local Planner Class"
    def __init__(self,replan_freq,goal_freq,fake_dyn_freq):
        
        # Storage
        # Inputs
        self.state = State()
        self.glob_plan = Path()
        self.cvx_decomp = CvxDecomp()
        self.global_goal = PoseStamped()
        self.master_node_state = MasterNodeState()

        # Flags for inputs being initialized
        self.received_state = False
        self.received_glob_plan = False
        self.received_cvx_decomp = False
        self.received_global_goal = False

        # Outputs
        self.goal = Goal()
        self.goal.p.x=0.0
        self.goal.p.y=0.0
        self.goal.p.z=0.0
        self.goal.v.x=0.0
        self.goal.v.y=0.0
        self.goal.v.z=0.0
        self.goal.a.x=0.0
        self.goal.a.y=0.0
        self.goal.a.z=0.0

        self.local_plan = Path()

        self.frame_id = "world"

        # Planner parameters
        # Run rates
        self.replan_freq = replan_freq
        self.goal_freq = goal_freq
        self.fake_dynamics_freq = fake_dyn_freq

        # Locks
        self.trajectory_lock = threading.Lock()

        # Vehicle limits
        self.v_max = rospy.get_param("~lp/v_max") # m/s
        self.a_max = rospy.get_param("~lp/a_max") # m/s2
        self.j_max = rospy.get_param("~lp/j_max") # m/s3

        # Path parameterization
        self.spline_deg = 3
        self.n_seg = rospy.get_param("~lp/n_seg")
        self.n_cp = self.spline_deg+1

        # Optimizer parameters
        self.coll_M = 1000 # Collision constraint upper bound
        self.n_int_max = rospy.get_param("~lp/n_int_max") # Maximum number of JPS intervals
        self.n_plane_max = rospy.get_param("~lp/n_plane_max") # Maximum number of polyhedron planes per interval
        self.log_settings = mosek.streamtype.wrn # Detail level of output
        self.set_var_names = True # Label variable names or not. Performance is allegedly faster without

        self.plane_norm_zero_tol = 1E-1

        # Time line search parameters
        self.f = 2.25 # Factor on constant motion solution. Start conservatively
        self.f_min = 1.25
        self.f_max = 5
        self.gamma_up = 0.75 # Limit on how much time factor can increase/decrease on each timestep
        self.gamma_down = 0.25
        self.gamma_step = 0.25
        self.perform_line_search = True
        self.max_line_search_it = 5
        self.last_soln_good = False
        self.inst_capability_usage = 0
        self.inst_capability_usage_margin = 0.1 # Margin to keep on v/a/j usage when decreasing time scale factor to prevent infeasibility
        self.n_int_capability_margin = 5 # How many intervals to check the capability margin condition over
        self.t_replan_margin = 0.05 # Margin on allowable replanning time to use when performing line search

        # Start MOSEK and initialize variables
        self.msk_env = mosek.Env()
        self.task = self.msk_env.Task()
        self.task_feas_check = self.msk_env.Task()
        self.task.puttaskname('Local Planner MIQP')
        self.task_feas_check.puttaskname('Local Planner Feasibility Check')

        # Solver settings
        self.task.putintparam(mosek.iparam.mio_heuristic_level,0)
        self.task_feas_check.putintparam(mosek.iparam.mio_heuristic_level,0)

        # Misc logging settings
        self.opt_run = False
        self.msk_env.set_Stream(self.log_settings, self.streamprinter) # Optimizer output
        self.task.set_Stream(self.log_settings, self.streamprinter) # Optimizer output
        self.task_feas_check.set_Stream(self.log_settings, self.streamprinter) # Optimizer output
        self.task.putintparam(mosek.iparam.infeas_report_auto, mosek.onoffkey.on) # Infeasibility report
        self.task_feas_check.putintparam(mosek.iparam.infeas_report_auto, mosek.onoffkey.on) # Infeasibility report
        self.task.putintparam(mosek.iparam.max_num_warnings,1) # Warning suppression(zeroing out constraints creates warnings)
        self.task_feas_check.putintparam(mosek.iparam.max_num_warnings,1) # Warning suppression(zeroing out constraints creates warnings)

        self.task.putintparam(mosek.iparam.opf_write_solutions,mosek.onoffkey.off)

        self.goal_log = True
        self.goal_log_int = 100
        self.goal_log_count = 0

        # Decision variables:
        # 4x position control points per segment,
        # 3x velocity, 2x acceleration, 1x jerk
        # 3x dimensions
        # n_int*n_seg binary variables
        self.num_pos = 3*self.n_seg*self.n_cp
        self.num_vel = 3*self.n_seg*(self.n_cp-1)
        self.num_accel = 3*self.n_seg*(self.n_cp-2)
        self.num_jerk = 3*self.n_seg*(self.n_cp-3)
        self.num_bin = self.n_int_max*self.n_seg

        self.numvar = self.num_pos+self.num_vel+self.num_accel+self.num_jerk+self.num_bin
        self.numvar_feas = self.num_pos + self.num_bin

        # Calculate indices of various locations for easy reference later
        self.ind_vel = self.num_pos
        self.ind_accel = self.ind_vel+self.num_vel
        self.ind_jerk = self.ind_accel+self.num_accel
        self.ind_bin = self.ind_jerk+self.num_jerk

        self.ind_bin_feas = self.num_pos

        # Constraints:
        # Listed in the following order: Continuity, collision, consistency between p/v/a/j, binary variable sum
        self.num_cont = 3*self.spline_deg*(self.n_seg-1)
        self.num_planes = self.n_int_max*self.n_plane_max
        self.num_coll_per_seg = self.n_cp*self.num_planes
        self.num_coll = self.n_seg*self.num_coll_per_seg
        self.num_cons = int(round(self.n_seg*3*self.spline_deg*(self.spline_deg+1)/2))
        self.num_bin_sum = self.n_seg

        self.num_cont_feas = 3*(self.n_seg-1)

        self.numcon = int(round(self.num_cont+self.num_coll+self.num_cons+self.num_bin_sum))
        self.numcon_feas = int(round(self.num_cont_feas + self.num_coll + self.num_bin_sum))

        # Calculate indices of various constraints for ease of reference later
        self.ind_coll = self.num_cont
        self.ind_cons = self.ind_coll+self.num_coll
        self.ind_bin_sum = self.ind_cons+self.num_cons

        self.ind_cont_pos = 0
        self.ind_cont_vel = 3*(self.n_seg-1)
        self.ind_cont_accel = self.ind_cont_vel+3*(self.n_seg-1)

        self.ind_cons_vel = self.ind_cons
        self.ind_cons_accel = self.ind_cons_vel+(self.n_cp-1)*3*self.n_seg
        self.ind_cons_jerk = self.ind_cons_accel+(self.n_cp-2)*3*self.n_seg

        self.ind_coll_feas = self.num_cont_feas
        self.ind_bin_sum_feas = self.ind_coll_feas + self.num_coll

        # Add variables
        self.task.appendvars(self.numvar)
        self.task.appendcons(self.numcon)

        self.task_feas_check.appendvars(self.numvar_feas)
        self.task_feas_check.appendcons(self.numcon_feas)

        # Storage for solution
        self.xx = np.zeros(self.numvar)
        self.cp_p_opt = np.zeros(self.num_pos)
        self.cp_v_opt = np.zeros(self.num_vel)
        self.cp_a_opt = np.zeros(self.num_accel)
        self.cp_j_opt = np.zeros(self.num_jerk)
        self.bin_opt = np.zeros(self.num_bin)
        self.soln_no_opt = 0

        self.t_traj_opt_start = 0
        self.dT_traj_opt = 10
        self.t_alloc_min = 0.1 # Minimum trajectory time s

        # Trajectory Parameters
        # Committed trajectory. We maintain this in case the optimizer does not converge for a few iterations, or if it finishes planning early
        self.cp_p_comm = np.zeros(self.num_pos)
        self.cp_v_comm = np.zeros(self.num_vel)
        self.cp_a_comm = np.zeros(self.num_accel)
        self.cp_j_comm = np.zeros(self.num_jerk)
        self.bin_comm = np.zeros(self.num_bin)
        self.soln_no_comm = 0

        self.t_traj_comm_start = 0
        self.dT_traj_comm = 10 # Nonzero value, avoid zero division

        # Parameters for goal and yaw tolerances for handling starting/stopping conditions
        # Solver is unstable when you're too close to the goal and camera isn't going to provide good images if you're not roughly pointed towards the goal
        self.goal_pos_tol = 2E-1
        self.goal_cp_tol = 1E-4
        self.goal_FOV = rospy.get_param("~lp/goal_FOV")*pi/180 # rad

        self.reached_goal = False
        self.goal_in_view = False

        self.yaw_filt_val = 0
        self.yaw_tol = 1E-2
        self.yaw_filt_cutoff = 10 # Hz
        self.yaw_filt_coef = (2*pi*self.yaw_filt_cutoff/self.goal_freq)/(1+2*pi*self.yaw_filt_cutoff/self.goal_freq)
        self.yaw_rate_max = 360*pi/180 # rad/s
        self.yaw_rate_find_goal = 45*pi/180 # rad/s
        self.yaw_change_max = self.yaw_rate_max/self.goal_freq
        self.yaw_change_find_goal_max = self.yaw_rate_find_goal/self.goal_freq

        # Option to plan from future or last reported state
        self.plan_start_future = True
        self.plan_future_t_fac = 1.25 # Factor to apply to execution time from last replanning iteration. Make sure we can find a new plan in time
        self.replan_time_prev = 0 # First replan should be immediate

        # Fake dynamics parameters
        self.fake_IMU = np.zeros(3)

        # Data logging
        self.replan_successful = False
        self.solve_times = []
        self.solve_times_reported = False

        # Start time
        self.t_start_int_debug = rospy.get_rostime()
        self.first_plan = True
        self.fake_plan = False # Test option for just executing plan from first state
        self.fake_t_alloc = 15
        self.t_start_plan = 0

        # Set variable and constraint names
        if self.set_var_names:
            # Variables
            ax_lab = ['x','y','z']
            start_inds = [0,self.ind_vel,self.ind_accel,self.ind_jerk]
            labels = ['p','v','a','j']

            # Spline control point variables
            # Organized as position, velocity, acceleration, jerk
            # Within each block ordered by control point, then axis, then segment
            # ie p_3_1_x = 3rd position control point for first spline segment in x
            for i in range(len(start_inds)):
                for j in range(self.n_seg):
                    for k in range(3): # 3D
                        for l in range(self.n_cp-i):
                            self.task.putvarname(start_inds[i]+j*3*(self.n_cp-i)+k*(self.n_cp-i)+l,
                                            '{}_{}_{}_{}'.format(labels[i],l,j,ax_lab[k]))
            
            # Binary interval allocation variables
            # Blocked by segment with one variable for each interval indicating if it's allocated to it
            # ie b_1_5 = if segment 1 belongs to interval 5
            for i in range(self.n_seg):
                for j in range(self.n_int_max):
                    self.task.putvarname(self.ind_bin+i*self.n_int_max+j,'b_{}_{}'.format(i,j))

            # Constraints
            # Continuity
            # Organized by position, then velocity, then acceleration
            # Each segment will have a constraint in x/y/z
            # ie cont_v_3_z = velocity continuity constraint for 3rd segment in z
            cont_ind_start = [self.ind_cont_pos,self.ind_cont_vel,self.ind_cont_accel]
            for i in range(len(cont_ind_start)):
                for j in range(self.n_seg-1):
                    for k in range(len(ax_lab)):
                        self.task.putconname(cont_ind_start[i]+j*3+k,
                                        'cont_{}_{}_{}'.format(labels[i],j,ax_lab[k]))
            
            # Collision Inequality
            # Block by curve segments, then control points within each segment, then intervals segment could belong to
            # then half planes that apply within that interval
            # ie coll_3_2_5_0 = Collision constraint for 2nd control point of third segment against the 0th plane of the 5th interval
            for i in range(self.n_seg):
                for j in range(self.n_cp):
                    for k in range(self.n_int_max):
                        for l in range(self.n_plane_max):
                            ind_coll_this = self.ind_coll + self.n_plane_max*(k+self.n_int_max*(j+self.n_cp*i)) + l
                            self.task.putconname(ind_coll_this,
                                            'coll_{}_{}_{}_{}'.format(i,j,k,l))
            
            # Velocity/Acceleration/Jerk Consistency
            # Enforce that v = delta p / delta t etc for spline control points
            # Block by velocity/acceleration/jerk. Then, for each segment order by control points followed by axis
            # ie cons_a_0_3_y = Acceleration consistency for 0th control point of 3rd segment in y direction
            cons_ind_start = [self.ind_cons_vel,self.ind_cons_accel,self.ind_cons_jerk]
            for i in range(len(cons_ind_start)):
                for j in range(self.n_seg):
                    for k in range(3):
                        for l in range(self.n_cp-i-1):
                            self.task.putconname(cons_ind_start[i]+j*3*(self.n_cp-i-1)+k*(self.n_cp-i-1)+l,
                                            'cons_{}_{}_{}_{}'.format(labels[i+1],l,j,ax_lab[k]))
            
            # Binary Variables
            # Enforce that sum of binary variables for each segment is greater than or equal to 1
            # ie a segment is allocated to at least one interval
            for i in range(self.n_seg):
                self.task.putconname(self.ind_bin_sum+i,'bin_sum_{}'.format(i))


            # Set for feasibility solution as well
            # Spline control point variables
            # Organized as position ordered by control point, then axis, then segment
            # ie p_3_1_x = 3rd position control point for first spline segment in x
            for j in range(self.n_seg):
                for k in range(3): # 3D
                    for l in range(self.n_cp):
                        self.task_feas_check.putvarname(j*3*self.n_cp+k*self.n_cp+l,
                                                        'p_{}_{}_{}'.format(l,j,ax_lab[k]))

            # Binary interval allocation variables
            # Blocked by segment with one variable for each interval indicating if it's allocated to it
            # ie b_1_5 = if segment 1 belongs to interval 5
            for i in range(self.n_seg):
                for j in range(self.n_int_max):
                    self.task_feas_check.putvarname(self.ind_bin_feas+i*self.n_int_max+j,'b_{}_{}'.format(i,j))

            # Constraints
            # Continuity. Each segment will have a constraint in x/y/z
            # ie cont_p_3_z = position continuity constraint for 3rd segment in z
            for j in range(self.n_seg-1):
                for k in range(len(ax_lab)):
                    self.task_feas_check.putconname(self.ind_cont_pos+j*3+k,
                                    'cont_p_{}_{}'.format(j,ax_lab[k]))
            
            # Collision Inequality
            # Block by curve segments, then control points within each segment, then intervals segment could belong to
            # then half planes that apply within that interval
            # ie coll_3_2_5_0 = Collision constraint for 2nd control point of third segment against the 0th plane of the 5th interval
            for i in range(self.n_seg):
                for j in range(self.n_cp):
                    for k in range(self.n_int_max):
                        for l in range(self.n_plane_max):
                            ind_coll_this = self.ind_coll_feas + self.n_plane_max*(k+self.n_int_max*(j+self.n_cp*i)) + l
                            self.task_feas_check.putconname(ind_coll_this,
                                            'coll_{}_{}_{}_{}'.format(i,j,k,l))

            # Binary Variables
            # Enforce that sum of binary variables for each segment is greater than or equal to 1
            # ie a segment is allocated to at least one interval
            for i in range(self.n_seg):
                self.task_feas_check.putconname(self.ind_bin_sum_feas+i,'bin_sum_{}'.format(i))

        # Initialize constraint data for decision variables and linear constraints
        # Many of these will not change from run to run
        # ICs/BCs and which binary variables/plane constraints are active will be the main ones

        # Decision variable bounds
        self.bkx = [mosek.boundkey.fr]*self.numvar
        self.blx = np.zeros(self.numvar)
        self.bux = np.zeros(self.numvar)

        # Set variable bound keys
        self.bkx[self.ind_vel:self.ind_accel] = [mosek.boundkey.ra]*self.num_vel
        self.bkx[self.ind_accel:self.ind_jerk] = [mosek.boundkey.ra]*self.num_accel
        self.bkx[self.ind_jerk:self.ind_bin] = [mosek.boundkey.ra]*self.num_jerk
        self.bkx[self.ind_bin:self.ind_bin+self.num_bin] = [mosek.boundkey.ra]*self.num_bin

        # Speed/acceleration/jerk bounds are dynamic limits
        self.blx[self.ind_vel:self.ind_accel] = -self.v_max
        self.bux[self.ind_vel:self.ind_accel] = self.v_max
        self.blx[self.ind_accel:self.ind_jerk] = -self.a_max
        self.bux[self.ind_accel:self.ind_jerk] = self.a_max
        self.blx[self.ind_jerk:self.ind_bin] = -self.j_max
        self.bux[self.ind_jerk:self.ind_bin] = self.j_max
        
        # Binary variables are 0/1
        self.blx[self.ind_bin:self.ind_bin+self.num_bin] = 0
        self.bux[self.ind_bin:self.ind_bin+self.num_bin] = 1
        self.task.putvartypelist(np.arange(self.ind_bin,self.ind_bin+self.num_bin),[mosek.variabletype.type_int]*self.num_bin)

        # Put the computed variable bounds on the task
        self.task.putvarboundslice(0,self.numvar,self.bkx,self.blx,self.bux)

        # Feasibility task variable bounds only exist for positions and binary variables
        bkx_feas = [mosek.boundkey.fr]*self.num_pos
        blx_feas = np.zeros(self.num_pos)
        bux_feas = blx_feas
        self.task_feas_check.putvarboundslice(0,self.num_pos,bkx_feas,blx_feas,bux_feas)
        self.task_feas_check.putvartypelist(np.arange(self.ind_bin_feas,self.ind_bin_feas+self.num_bin),[mosek.variabletype.type_int]*self.num_bin)
        self.task_feas_check.putvarboundslice(self.ind_bin_feas,self.numvar_feas,
                                            [mosek.boundkey.ra]*self.num_bin,np.zeros(self.num_bin),np.ones(self.num_bin))

        # Constraints
        self.bkc = [mosek.boundkey.fr]*self.numcon
        self.blc = np.zeros(self.numcon)
        self.buc = np.zeros(self.numcon)

        # Constraint bound keys
        # Continuity constraints get clamped to zero
        self.bkc[0:self.num_cont] = [mosek.boundkey.fx]*self.num_cont
        # Half plane collision constraint keys will be set every loop as number of planes changes
        # All consistency constraints are equality constraints
        self.bkc[self.ind_cons:self.ind_bin_sum] = [mosek.boundkey.fx]*self.num_cons
        # Binary sum constraints are lower bounds
        self.bkc[self.ind_bin_sum:self.ind_bin_sum+self.num_bin_sum] = [mosek.boundkey.lo]*self.num_bin_sum

        # Continuity
        # Position
        a_val_pos = [1,-1]*3*(self.n_seg-1)
        a_sub_pos_j = list(range(self.n_cp-1,3*self.n_cp*(self.n_seg-1),self.n_cp))
        a_sub_pos_jp1 = list(range(3*self.n_cp,3*self.n_cp*(self.n_seg-1)+2*self.n_cp+1,self.n_cp))
        a_sub_pos = [None]*len(a_val_pos)
        a_sub_pos[::2] = a_sub_pos_j
        a_sub_pos[1::2] = a_sub_pos_jp1
        ptrb_pos = list(range(0,len(a_val_pos),2))
        ptre_pos = list(range(2,len(a_val_pos)+1,2))

        self.task.putarowslice(self.ind_cont_pos,self.ind_cont_vel,
                            ptrb_pos,ptre_pos,a_sub_pos,a_val_pos)
        
        # Velocity
        a_val_vel = [1,-1]*3*(self.n_seg-1)
        a_sub_vel_j = list(range(self.ind_vel+self.n_cp-2,
                                    self.ind_vel+3*(self.n_cp-1)*(self.n_seg-1),self.n_cp-1))
        a_sub_vel_jp1 = list(range(self.ind_vel+3*(self.n_cp-1),
                                    self.ind_vel+3*(self.n_cp-1)*(self.n_seg-1)+2*(self.n_cp-1)+1,self.n_cp-1))
        a_sub_vel = [None]*len(a_val_vel)
        a_sub_vel[::2] = a_sub_vel_j
        a_sub_vel[1::2] = a_sub_vel_jp1
        ptrb_vel = list(range(0,len(a_val_vel),2))
        ptre_vel = list(range(2,len(a_val_vel)+1,2))
        
        self.task.putarowslice(self.ind_cont_vel,self.ind_cont_accel,
                            ptrb_vel,ptre_vel,a_sub_vel,a_val_vel)
        
        # Acceleration
        a_val_acc = [1,-1]*3*(self.n_seg-1)
        a_sub_acc_j = list(range(self.ind_accel+self.n_cp-3,
                                    self.ind_accel+3*(self.n_cp-2)*(self.n_seg-1),self.n_cp-2))
        a_sub_acc_jp1 = list(range(self.ind_accel+3*(self.n_cp-2),
                                    self.ind_accel+3*(self.n_cp-2)*(self.n_seg-1)+2*(self.n_cp-2)+1,self.n_cp-2))
        a_sub_acc = [None]*len(a_val_acc)
        a_sub_acc[::2] = a_sub_acc_j
        a_sub_acc[1::2] = a_sub_acc_jp1
        ptrb_acc = list(range(0,len(a_val_acc),2))
        ptre_acc = list(range(2,len(a_val_acc)+1,2))
        
        self.task.putarowslice(self.ind_cont_accel,self.ind_coll,
                            ptrb_acc,ptre_acc,a_sub_acc,a_val_acc)

        # No point in putting anything for collision. It'll change on the first iteration
        # Velocity/Acceleration/Jerk Consistency
        # Order by control point, coordinate, then segment
        # Velocity
        a_val_v_cons = np.zeros((self.num_vel,2))
        
        a_val_v_cons[:,0] = 3*np.ones(self.num_vel)
        a_val_v_cons[:,1] = -3*np.ones(self.num_vel)
        
        ind_p0 = np.arange(0,self.num_pos,self.n_cp,dtype=np.int32)
        
        # Sparsity pattern for this constraint. See documentation
        # Full constraint has a term for velocity as well that has a factor of dT
        # This is added on each timestep as it changes
        a_sub_v_cons = np.zeros(np.size(a_val_v_cons),dtype=np.int32)
        a_sub_v_cons[0::2*(self.n_cp-1)] = ind_p0
        a_sub_v_cons[1::2*(self.n_cp-1)] = ind_p0+1
        a_sub_v_cons[2::2*(self.n_cp-1)] = ind_p0+1
        a_sub_v_cons[3::2*(self.n_cp-1)] = ind_p0+2
        a_sub_v_cons[4::2*(self.n_cp-1)] = ind_p0+2
        a_sub_v_cons[5::2*(self.n_cp-1)] = ind_p0+3
        
        ptrb_v_cons = np.arange(0,np.size(a_val_v_cons),2,dtype=np.int32)
        ptre_v_cons = np.arange(2,np.size(a_val_v_cons)+1,2,dtype=np.int32)
        
        # Constraint is set to zero
        self.blc[self.ind_cons_vel:self.ind_cons_accel] = np.zeros(self.num_vel)
        self.buc[self.ind_cons_vel:self.ind_cons_accel] = np.zeros(self.num_vel)
        
        # Put coefficients for constraint
        self.task.putarowslice(self.ind_cons_vel,self.ind_cons_accel,ptrb_v_cons,ptre_v_cons
                            ,a_sub_v_cons,a_val_v_cons.flatten())

        # Acceleration        
        a_val_a_cons = np.zeros((self.num_accel,2))
        
        a_val_a_cons[:,0] = 2*np.ones(self.num_accel)
        a_val_a_cons[:,1] = -2*np.ones(self.num_accel)
        
        ind_v0 = np.arange(self.ind_vel,self.ind_accel,self.n_cp-1,dtype=np.int32)
        
        # Sparsity pattern for this constraint. See documentation
        # Again, factor of dT changes for each timestep so only set the constant terms
        a_sub_a_cons = np.zeros(np.size(a_val_a_cons),dtype=np.int32)
        a_sub_a_cons[0::2*(self.n_cp-2)] = ind_v0
        a_sub_a_cons[1::2*(self.n_cp-2)] = ind_v0+1
        a_sub_a_cons[2::2*(self.n_cp-2)] = ind_v0+1
        a_sub_a_cons[3::2*(self.n_cp-2)] = ind_v0+2
        
        ptrb_a_cons = np.arange(0,np.size(a_val_a_cons),2,dtype=np.int32)
        ptre_a_cons = np.arange(2,np.size(a_val_a_cons)+1,2,dtype=np.int32)
        
        # Constraint is set to zero
        self.blc[self.ind_cons_accel:self.ind_cons_jerk] = np.zeros(self.num_accel)
        self.buc[self.ind_cons_accel:self.ind_cons_jerk] = np.zeros(self.num_accel)
        
        # Put coefficients for constraint
        self.task.putarowslice(self.ind_cons_accel,self.ind_cons_jerk,ptrb_a_cons,ptre_a_cons
                            ,a_sub_a_cons,a_val_a_cons.flatten())
        
        # Jerk
        a_val_j_cons = np.zeros((self.num_jerk,2))
        
        a_val_j_cons[:,0] = np.ones(self.num_jerk)
        a_val_j_cons[:,1] = -1*np.ones(self.num_jerk)
        
        ind_a0 = np.arange(self.ind_accel,self.ind_jerk,self.n_cp-2,dtype=np.int32)
        
        # Sparsity pattern for this constraint. See documentation
        a_sub_j_cons = np.zeros(np.size(a_val_j_cons),dtype=np.int32)
        a_sub_j_cons[0::2*(self.n_cp-3)] = ind_a0
        a_sub_j_cons[1::2*(self.n_cp-3)] = ind_a0+1
        
        ptrb_j_cons = np.arange(0,np.size(a_val_j_cons),2,dtype=np.int32)
        ptre_j_cons = np.arange(2,np.size(a_val_j_cons)+1,2,dtype=np.int32)
        
        # Constraint is to zero
        self.blc[self.ind_cons_jerk:self.ind_bin_sum] = np.zeros(self.num_jerk)
        self.buc[self.ind_cons_jerk:self.ind_bin_sum] = np.zeros(self.num_jerk)
        
        # Put coefficients for constraint
        self.task.putarowslice(self.ind_cons_jerk,self.ind_bin_sum,ptrb_j_cons,ptre_j_cons
                            ,a_sub_j_cons,a_val_j_cons.flatten())

        # Binary variable sum constraints
        # Simply just ones in a row for each segment's set of binary variables
        a_sub_b_sum = np.arange(self.ind_bin,self.ind_bin+self.num_bin)
        a_val_b_sum = np.ones(self.num_bin)
        ptrb_b_sum = np.arange(0,self.num_bin,self.n_int_max)
        ptre_b_sum = np.arange(self.n_int_max,self.num_bin+1,self.n_int_max)
        
        # Sum must be at least one
        self.blc[self.ind_bin_sum:(self.ind_bin_sum+self.n_seg)] = np.ones(self.n_seg)
        
        # Put coefficients for constraint
        self.task.putarowslice(self.ind_bin_sum,self.ind_bin_sum+self.num_bin_sum,ptrb_b_sum,ptre_b_sum,a_sub_b_sum,a_val_b_sum)
        
        # Set constraint bounds
        self.task.putconboundslice(0,self.numcon,self.bkc,self.blc,self.buc)

        # Cost function changes weakly with timestep. Just quadratic with dTs on the diagonal for jerk decision variables

        # Feasibility task constraints
        bkc_feas = [mosek.boundkey.fr]*self.numcon_feas
        blc_feas = np.zeros(self.numcon_feas)
        buc_feas = np.zeros(self.numcon_feas)

        # Constraint bound keys
        # Continuity constraints get clamped to zero
        bkc_feas[0:self.num_cont_feas] = [mosek.boundkey.fx]*self.num_cont_feas
        # Half plane collision constraint keys will be set every loop as number of planes changes
        # Binary sum constraints are lower bounds
        bkc_feas[self.ind_bin_sum_feas:self.ind_bin_sum_feas+self.num_bin_sum] = [mosek.boundkey.lo]*self.num_bin_sum

        # Continuity
        # Position. Identical to full problem
        self.task_feas_check.putarowslice(self.ind_cont_pos,self.ind_cont_vel,
                            ptrb_pos,ptre_pos,a_sub_pos,a_val_pos)

        # Binary variable sum constraints
        # Simply just ones in a row for each segment's set of binary variables
        a_sub_b_sum_feas = np.arange(self.ind_bin_feas,self.ind_bin_feas+self.num_bin)
        
        # Sum must be at least one
        blc_feas[self.ind_bin_sum_feas:(self.ind_bin_sum_feas+self.n_seg)] = np.ones(self.n_seg)
        
        # Put coefficients for constraint
        self.task_feas_check.putarowslice(self.ind_bin_sum_feas,self.ind_bin_sum_feas+self.num_bin_sum,
                                        ptrb_b_sum,ptre_b_sum,a_sub_b_sum_feas,a_val_b_sum)

        # Set constraint bounds
        self.task_feas_check.putconboundslice(0,self.numcon_feas,bkc_feas,blc_feas,buc_feas)

        # Define objective function sense
        self.task.putobjsense(mosek.objsense.minimize)
        self.task_feas_check.putobjsense(mosek.objsense.minimize)

    def __del__(self):
        # Close MOSEK
        self.task.__del__()
        self.task_feas_check.__del__()
        self.msk_env.__del__()

    def replan(self):
        # Reset replan successful indicator
        self.replan_successful = False

        # Check inputs initialized
        if not (self.received_state and self.received_glob_plan and 
                self.received_cvx_decomp and self.received_global_goal and self.master_node_state.state == self.master_node_state.FLIGHT_LOCAL):
            return

        # Get start time
        t_replan_start = rospy.get_rostime()
        t_replan_start_s = t_replan_start.to_sec()

        # Option for fake planning where we execute static problem from first state
        if self.fake_plan and self.first_plan:
            self.t_start_plan = t_replan_start.to_sec()
            self.first_plan = False

        # TODO: May need to consider thread safety
        state_start = copy.deepcopy(self.state)
        global_plan = copy.deepcopy(self.glob_plan)
        cvx_decomp = copy.deepcopy(self.cvx_decomp)

        # See if we've reached the goal within tolerance. If so don't need to keep planning, solver can be unstable and yaw angle can go crazy
        d_goal = np.sqrt((state_start.pos.x - self.global_goal.pose.position.x)**2 +
                          (state_start.pos.y - self.global_goal.pose.position.y)**2 +
                          (state_start.pos.z - self.global_goal.pose.position.z)**2)
        d_goal_cp = np.sqrt((self.cp_p_comm[3*self.n_cp*(self.n_seg-1)+(self.n_cp-1)] - self.global_goal.pose.position.x)**2 +
                          (self.cp_p_comm[3*self.n_cp*(self.n_seg-1)+(2*self.n_cp-1)] - self.global_goal.pose.position.y)**2 +
                          (self.cp_p_comm[3*self.n_cp*(self.n_seg-1)+(3*self.n_cp-1)] - self.global_goal.pose.position.z)**2)
        self.reached_goal = d_goal < self.goal_pos_tol and d_goal_cp < self.goal_cp_tol

        if self.reached_goal:
            return

        # Check if we can see goal within tolerance. Stop come to a stop and rotate until we can see goal
        heading_goal = np.arctan2(self.global_goal.pose.position.y-state_start.pos.y,self.global_goal.pose.position.x-state_start.pos.x)
        quat_quad = [state_start.quat.x,state_start.quat.y,state_start.quat.z,state_start.quat.w]
        euler_quad = euler_from_quaternion(quat_quad,'rzyx')
        self.goal_in_view = np.abs(heading_goal - euler_quad[0])<(self.goal_FOV/2)

        if not self.goal_in_view:
            return

        # Problem size
        # Clip to maximum number of intervals and planes
        n_int_poly = min(len(cvx_decomp.polyhedra),self.n_int_max)
        n_int_glob = min(len(global_plan.poses)-1,self.n_int_max)
        n_int = min(n_int_poly,n_int_glob)
        if n_int_poly <= 0 or n_int_glob <= 0:
            rospy.loginfo('Invalid global path or convex decomposition length. Global:{}, CvxDecomp:{}'.format(n_int_glob,n_int_poly))
            return        
        elif not(n_int==n_int_glob):
            rospy.loginfo("Number of polyhedra does not match global path length({}~={})".format(n_int,n_int_glob))
        n_planes = np.zeros(n_int,dtype = np.int32)
        for i in range(n_int):
            n_planes[i] = min(len(cvx_decomp.polyhedra[i].planes),self.n_plane_max)

        # Calculate initial condition
        if self.fake_plan:
            # For fake static problem just run from first state
            x_0 = np.array([global_plan.poses[0].pose.position.x,
                            global_plan.poses[0].pose.position.y,
                            global_plan.poses[0].pose.position.z])
            v_0 = np.zeros(3)
            a_0 = np.zeros(3)
        else:
            # Calculate time to plan from
            if self.plan_start_future:
                t_plan_start = t_replan_start_s + self.plan_future_t_fac*self.replan_time_prev
            else:
                t_plan_start = state_start.header.stamp.to_sec()

            # Skip this replanning step if haven't begun to use last optimal trajectory. Saves a ton of overhead
            # TODO: This could be improved
            if not t_replan_start_s > self.t_traj_opt_start:
                return
            
            # Position and velocity depends on if we plan from current or future state
            if self.plan_start_future and self.opt_run:
                # Interpolate either committed or most recent optimal trajectory result, depending on which is appropriate
                if t_plan_start > self.t_traj_opt_start:
                    # Interpolate on most recent optimal trajectory result
                    x_0 = bezier_interpolate(self.cp_p_opt.reshape(self.n_cp,3,self.n_seg,order='F').swapaxes(0,1),
                                            self.dT_traj_opt*np.ones(self.n_seg),t_plan_start-self.t_traj_opt_start,n=3).flatten()
                    v_0 = bezier_interpolate(self.cp_v_opt.reshape(self.n_cp-1,3,self.n_seg,order='F').swapaxes(0,1),
                                            self.dT_traj_opt*np.ones(self.n_seg),t_plan_start-self.t_traj_opt_start,n=2).flatten()
                else:
                    # Won't have yet reached most recent optimal trajectory result, stick to committed trajectory
                    # TODO: Think can delete this case
                    x_0 = bezier_interpolate(self.cp_p_comm.reshape(self.n_cp,3,self.n_seg,order='F').swapaxes(0,1),
                                            self.dT_traj_comm*np.ones(self.n_seg),t_plan_start-self.t_traj_comm_start,n=3).flatten()
                    v_0 = bezier_interpolate(self.cp_v_comm.reshape(self.n_cp-1,3,self.n_seg,order='F').swapaxes(0,1),
                                            self.dT_traj_comm*np.ones(self.n_seg),t_plan_start-self.t_traj_comm_start,n=2).flatten()
            else:
                # Plan from current state, rather than a future point
                x_0 = np.array([state_start.pos.x,state_start.pos.y,state_start.pos.z])
                v_0 = np.array([state_start.vel.x,state_start.vel.y,state_start.vel.z])
            
            # Interpolate acceleration at planning start time, no need to distinguish
            if not self.opt_run:
                # Initialize with zeros
                a_0 = np.zeros(3)
            elif t_plan_start > self.t_traj_opt_start:
                # Interpolate on most recent optimal trajectory result
                a_0_future = bezier_interpolate(self.cp_a_opt.reshape(self.n_cp-2,3,self.n_seg,order='F').swapaxes(0,1),
                                        self.dT_traj_opt*np.ones(self.n_seg),t_plan_start-self.t_traj_opt_start,n=1).flatten()
                a_0 = a_0_future
            else:
                # Won't have yet reached most recent optimal trajectory result, stick to committed trajectory
                # TODO: Think can delete this case
                a_0_future = bezier_interpolate(self.cp_a_comm.reshape(self.n_cp-2,3,self.n_seg,order='F').swapaxes(0,1),
                                        self.dT_traj_comm*np.ones(self.n_seg),t_plan_start-self.t_traj_comm_start,n=1).flatten()
                a_0 = a_0_future

        x_f = np.array([global_plan.poses[n_int].pose.position.x,
                        global_plan.poses[n_int].pose.position.y,
                        global_plan.poses[n_int].pose.position.z])
        v_f = np.zeros(3)
        a_f = np.zeros(3)

        # Update bounds and constraints based on parameters for this timestep
        # ICs/BCs via variable bounds
        # Velocity/position at initial condition, p/v/a at final condition (acceleration not defined in fake sim)
        # TODO: Need to get initial acceleration in here somehow. Assume zero as a placeholder for now
        bkx_ic_bc = [mosek.boundkey.fx]*3*6
        blx_ic_bc = np.hstack((x_0,v_0,a_0,x_f,v_f,a_f))
        bux_ic_bc = blx_ic_bc
        b_ind_ic_bc = []

        ic_all = np.vstack((x_0,v_0,a_0))
        bc_all = np.vstack((x_f,v_f,a_f))
        # Initial conditions
        ind_table = [0,self.ind_vel,self.ind_accel]
        for i in range(3):
            for j in range(3):
                ind_ic = ind_table[i]+j*(self.n_cp-i)
                b_ind_ic_bc.append(ind_ic)
                self.bkx[ind_ic] = mosek.boundkey.fx
                self.blx[ind_ic] = ic_all[i,j]
                self.bux[ind_ic] = ic_all[i,j]
        # Boundary conditions
        for i in range(3):
            for j in range(3):
                ind_bc = ind_table[i]+3*(self.n_seg-1)*(self.n_cp-i)+j*(self.n_cp-i)+self.n_cp-i-1
                b_ind_ic_bc.append(ind_bc)
                self.bkx[ind_bc] = mosek.boundkey.fx
                self.blx[ind_bc] = bc_all[i,j]
                self.bux[ind_bc] = bc_all[i,j]
        
        # Put bounds on task
        self.task.putvarboundlist(b_ind_ic_bc,bkx_ic_bc,blx_ic_bc,bux_ic_bc)

        # Feasibility problem needs these as well
        blx_ic_bc_feas = np.hstack((x_0,x_f))
        bux_ic_bc_feas = blx_ic_bc_feas
        bkx_ic_bc_feas = [mosek.boundkey.fx]*3*2
        b_ind_ic_bc_feas = []

        # Initial conditions
        for i in range(3):
            ind_ic = i*self.n_cp
            b_ind_ic_bc_feas.append(ind_ic)
        
        # Boundary conditions
        for i in range(3):
            ind_bc = 3*(self.n_seg-1)*self.n_cp+i*(self.n_cp)+self.n_cp-1
            b_ind_ic_bc_feas.append(ind_bc)

        # Put bounds on task
        self.task_feas_check.putvarboundlist(b_ind_ic_bc_feas,bkx_ic_bc_feas,blx_ic_bc_feas,bux_ic_bc_feas)

        # Collision Inequality
        # Each timestep is a brand new day, do not know what was/wasn't active. Remove all constraints then reenable ones needed
        # Zero out inequality constraints from last timestep, fix all binary variables to zero
        a_subj_coll_clear = np.zeros(self.num_coll,dtype=np.int32)
        a_val_coll_clear = np.zeros(self.num_coll)
        ptrb_coll_clear = np.arange(self.num_coll,dtype=np.int32)
        ptre_coll_clear = ptrb_coll_clear+1

        self.task.putarowslice(self.ind_coll,self.ind_cons,ptrb_coll_clear,ptre_coll_clear,
                                a_subj_coll_clear,a_val_coll_clear)
        self.task.putconboundslice(self.ind_coll,self.ind_cons,[mosek.boundkey.fr]*self.num_coll,
                                    np.zeros(self.num_coll),np.zeros(self.num_coll))
        self.bkc[self.ind_coll:self.ind_cons] = [mosek.boundkey.fr]*self.num_coll
        self.blc[self.ind_coll:self.ind_cons] = np.zeros(self.num_coll)
        self.buc[self.ind_coll:self.ind_cons] = np.zeros(self.num_coll)

        self.bkx[self.ind_bin:self.ind_bin+self.num_bin] = [mosek.boundkey.fx]*self.num_bin
        self.blx[self.ind_bin:self.ind_bin+self.num_bin] = np.zeros(self.num_bin)
        self.bux[self.ind_bin:self.ind_bin+self.num_bin] = np.zeros(self.num_bin)
        self.task.putvarboundslice(self.ind_bin,self.ind_bin+self.num_bin,[mosek.boundkey.fx]*self.num_bin,
                                    np.zeros(self.num_bin),np.zeros(self.num_bin))

        # Also reset constraints and variable bounds for feasibility checking problem
        self.task_feas_check.putarowslice(self.ind_coll_feas,self.ind_bin_sum_feas,ptrb_coll_clear,ptre_coll_clear,
                                a_subj_coll_clear,a_val_coll_clear)
        self.task_feas_check.putconboundslice(self.ind_coll_feas,self.ind_bin_sum_feas,[mosek.boundkey.fr]*self.num_coll,
                                    np.zeros(self.num_coll),np.zeros(self.num_coll))
        self.task_feas_check.putvarboundslice(self.ind_bin_feas,self.ind_bin_feas+self.num_bin,[mosek.boundkey.fx]*self.num_bin,
                                    np.zeros(self.num_bin),np.zeros(self.num_bin))

        # Now build collision constraints for this timestep
        # Extract plane coefficients from convex decomposition
        plane_norms = np.zeros((self.n_int_max,self.n_plane_max,3))
        plane_coefs = np.zeros((self.n_int_max,self.n_plane_max))
        plane_present = np.zeros((self.n_int_max,self.n_plane_max))
        for i in range(n_int):
            for j in range(n_planes[i]):
                if np.linalg.norm(np.asarray(cvx_decomp.polyhedra[i].planes[j].coef[0:3])) < self.plane_norm_zero_tol:
                    # Skip invalid planes
                    continue
                plane_norms[i,j,:] = cvx_decomp.polyhedra[i].planes[j].coef[0:3]
                plane_coefs[i,j] = cvx_decomp.polyhedra[i].planes[j].coef[3]
                plane_present[i,j] = 1
        coll_rel_row_inds = np.arange(self.n_plane_max*self.n_int_max,dtype=np.int32).reshape(plane_coefs.shape,order='C')
        coll_bin_rel_col_inds = np.tile(np.arange(self.n_int_max,dtype=np.int32),self.n_plane_max).reshape(plane_coefs.shape,order='F')

        # Extract nonzero coefficients
        ind_nz = np.nonzero(plane_present)
        num_coll_actv = np.size(ind_nz[0])*self.n_cp
        plane_norms_nz_x = plane_norms[ind_nz[0],ind_nz[1],0]
        plane_norms_nz_y = plane_norms[ind_nz[0],ind_nz[1],1]
        plane_norms_nz_z = plane_norms[ind_nz[0],ind_nz[1],2]
        plane_coefs_nz = plane_coefs[ind_nz[0],ind_nz[1]]
        coll_rel_nz_row_inds = coll_rel_row_inds[ind_nz[0],ind_nz[1]]
        coll_bin_rel_nz_col_inds = coll_bin_rel_col_inds[ind_nz[0],ind_nz[1]]

        for i in range(self.n_seg):
            # Loop over curve segments. All control points in each segment must be checked against all planes
            # along with binary variables to indicate which constraints are active                                 
            ind_coll_start = self.ind_coll+i*self.num_coll_per_seg
            ind_coll_end = ind_coll_start + self.num_coll_per_seg
            
            # Index row coefficients by running down each block of columns in turn
            # Requires 4 passes for 3x dimensions + 1x binary variables
            # TODO: Can probably broadcast this rather than tiling
            coll_subi = (np.tile(coll_rel_nz_row_inds,(self.n_cp,1))+
                        np.arange(ind_coll_start,ind_coll_end-1,self.num_planes,dtype=np.int32).reshape((self.n_cp,1))).flatten(order='C')
            coll_subi_rep = np.tile(coll_subi,4)
            coll_subi_rep_feas = coll_subi_rep + (self.ind_coll_feas-self.ind_coll)

            # Column indices need to be repeated num_planes times
            coll_subj_cps = np.tile(np.arange(i*3*self.n_cp,(i+1)*3*self.n_cp,dtype=np.int32),
                                    (ind_nz[0].shape[0],1)).flatten(order='F')

            # Column indices of binary coefficients. Stacking order has them along diagonals for each control point
            coll_subj_bin = np.tile(self.ind_bin+i*self.n_int_max+coll_bin_rel_nz_col_inds,self.n_cp)
            coll_subj_bin_feas = np.tile(self.ind_bin_feas+i*self.n_int_max+coll_bin_rel_nz_col_inds,self.n_cp)
            
            # Actual coefficients of constraint matrix are based on plane normals
            coll_a_x = np.tile(plane_norms_nz_x,self.n_cp)
            coll_a_y = np.tile(plane_norms_nz_y,self.n_cp)
            coll_a_z = np.tile(plane_norms_nz_z,self.n_cp)
            
            # Binary variables just get the M coefficient
            coll_a_bin = self.coll_M*np.ones(num_coll_actv)

            coll_b = np.tile(plane_coefs_nz,self.n_cp)+self.coll_M
            
            # Put the A matrix and RHS
            # RHS is simply plane coefficients repeated for each control point plus the M coefficient
            self.task.putaijlist(coll_subi_rep,np.hstack((coll_subj_cps,coll_subj_bin)),
                            np.hstack((coll_a_x,coll_a_y,coll_a_z,coll_a_bin)))
            self.task.putconboundlist(coll_subi,[mosek.boundkey.up]*num_coll_actv,
                                    np.zeros(num_coll_actv),coll_b)

            # Similar story for feasibility checking problem
            self.task_feas_check.putaijlist(coll_subi_rep_feas,np.hstack((coll_subj_cps,coll_subj_bin_feas)),
                            np.hstack((coll_a_x,coll_a_y,coll_a_z,coll_a_bin)))
            self.task_feas_check.putconboundlist(coll_subi+(self.ind_coll_feas-self.ind_coll),[mosek.boundkey.up]*num_coll_actv,
                                    np.zeros(num_coll_actv),coll_b)

            # Store constraint values and keys for reference
            # TODO: Had to unvectorize this b/c numpy was complaining. Probably is a way to do it though
            # self.bkc[coll_subi] = [mosek.boundkey.up]*num_coll_actv
            # self.buc[coll_subi] = np.tile(plane_coefs_nz,self.n_cp)+self.coll_M
            for j in range(coll_subi.shape[0]):
                self.bkc[coll_subi[j]] = [mosek.boundkey.up]
                self.buc[coll_subi[j]] = coll_b[j]

        
        # Enable active binary variables, store bounds/keys for reference
        ind_bin_actv = (np.arange(self.ind_bin,self.ind_bin+self.num_bin,self.n_int_max)
                        +np.arange(0,n_int).reshape((n_int,1))).flatten(order='F')
        n_bin_actv = np.size(ind_bin_actv)
        self.task.putvarboundlist(ind_bin_actv,[mosek.boundkey.ra]*n_bin_actv,
                                np.zeros(n_bin_actv),np.ones(n_bin_actv))
        self.task_feas_check.putvarboundlist(ind_bin_actv + (self.ind_bin_feas-self.ind_bin),
                                [mosek.boundkey.ra]*n_bin_actv,np.zeros(n_bin_actv),np.ones(n_bin_actv))
        
        # TODO: Again, unvectorized because of numpy complaining
        # self.bkx[ind_bin_actv] = [mosek.boundkey.ra]*n_bin_actv
        # self.blx[ind_bin_actv] = np.zeros(n_bin_actv)
        # self.bux[ind_bin_actv] = np.ones(n_bin_actv)
        for i in range(ind_bin_actv.shape[0]):
            self.bkx[ind_bin_actv[i]] = [mosek.boundkey.ra]
            self.blx[ind_bin_actv[i]] = 0
            self.bux[ind_bin_actv[i]] = 1
        
        # Call solver to check feasibility of given polyhedra
        self.task_feas_check.optimize()

        # Check solution status
        prosta_feas = self.task_feas_check.getprosta(mosek.soltype.itg)
        solsta_feas = self.task_feas_check.getsolsta(mosek.soltype.itg)

        if (solsta_feas == mosek.solsta.unknown or solsta_feas == mosek.solsta.prim_infeas_cer
            or solsta_feas == mosek.solsta.dual_infeas_cer or solsta_feas == mosek.solsta.prim_illposed_cer
            or solsta_feas == mosek.solsta.dual_illposed_cer):
            # Input polyhedra lead to an infeasible problem. Maintain current plan until receive a feasible set of inputs
            rospy.loginfo("Infeasible polyhedra: {}, {} at t = {:.1f}".format(str(prosta_feas), 
                                                                        str(solsta_feas),rospy.get_rostime().to_sec()))
            return

        # Grab bounds on time allocation factor for this replanning iteration
        f_max_curr = self.f+self.gamma_up
        f_min_curr = self.f-self.gamma_down

        # Time allocation line search loop
        for i in range(self.max_line_search_it):
            if self.perform_line_search and self.opt_run:
                # Dynamically adjust time allocation factor to find faster time trajectories
                # Time allocation factor is constant otherwise
                if self.last_soln_good:
                    if (self.inst_capability_usage < (1-self.inst_capability_usage_margin)):
                        # Last timestep successfully found a solution and had capability margin. Try to decrease the time allocation factor
                        # Else, maintain it
                        self.f = max(f_min_curr,self.f_min)
                else:
                    # Did not find a solution last iteration. Bump up time allocation factor
                    self.f = min(self.f+self.gamma_step,f_max_curr,self.f_max)

            # Calculate time allocation. Minimum time for constant input motions between current and goal states, multiply by adaptive scale factor
            dx = x_f-x_0
            dv = v_f-v_0
            da = a_f-a_0
            t_min = np.max(np.vstack((np.abs(dx)/self.v_max,
                                    np.abs(dv)/self.a_max,
                                    np.abs(da)/self.j_max)))
            t_alloc = t_min*self.f

            if self.fake_plan:
                # Time allocation fixed for fake static problem
                t_alloc = self.fake_t_alloc
            t_alloc = max(t_alloc,self.t_alloc_min) # Minimum trajectory time
            dT = t_alloc/self.n_seg

            # Velocity/Acceleration/Jerk Consistency - Just need to update time values
            # Velocity
            a_val_v_cons = dT*np.ones(self.num_vel)
            a_i_v_cons = np.arange(self.ind_cons_vel,self.ind_cons_accel,dtype=np.int32)
            a_j_v_cons = np.arange(self.ind_vel,self.ind_accel,dtype=np.int32)

            self.task.putaijlist(a_i_v_cons,a_j_v_cons,a_val_v_cons)

            # Acceleration
            a_val_a_cons = dT*np.ones(self.num_accel)
            a_i_a_cons = np.arange(self.ind_cons_accel,self.ind_cons_jerk,dtype=np.int32)
            a_j_a_cons = np.arange(self.ind_accel,self.ind_jerk,dtype=np.int32)

            self.task.putaijlist(a_i_a_cons,a_j_a_cons,a_val_a_cons)

            # Jerk
            a_val_j_cons = dT*np.ones(self.num_jerk)
            a_i_j_cons = np.arange(self.ind_cons_jerk,self.ind_bin_sum,dtype=np.int32)
            a_j_j_cons = np.arange(self.ind_jerk,self.ind_bin,dtype=np.int32)

            self.task.putaijlist(a_i_j_cons,a_j_j_cons,a_val_j_cons)

            # Set cost
            # Quadratic is norm of jerk times dT
            q_cost_subi = np.arange(self.ind_jerk,self.ind_bin)
            q_cost_val = 2*dT*np.ones(3*self.n_seg)
            self.task.putqobj(q_cost_subi,q_cost_subi,q_cost_val)

            # Call solver
            t_opt_start = rospy.get_rostime()
            self.task.optimize()
            self.task.solutionsummary(mosek.streamtype.log)
            self.task.optimizersummary(mosek.streamtype.log)

            # Check solution status
            prosta = self.task.getprosta(mosek.soltype.itg)
            solsta = self.task.getsolsta(mosek.soltype.itg)
            t_opt_finish = rospy.get_rostime()

            self.last_soln_good = not (solsta == mosek.solsta.unknown or solsta == mosek.solsta.prim_infeas_cer
                        or solsta == mosek.solsta.dual_infeas_cer or solsta == mosek.solsta.prim_illposed_cer
                        or solsta == mosek.solsta.dual_illposed_cer)

            t_soln = t_opt_finish-t_opt_start

            if self.last_soln_good:
                rospy.loginfo("Successful solution: {}, {}".format(str(prosta), str(solsta)))
                rospy.loginfo("At t = {:.1f} s. Solve time: {:.1f} ms. f: {:.2f}".format(t_opt_finish.to_sec(),
                                                                                1000*(t_soln.to_sec()),self.f))
                rospy.loginfo('x_0 = {}'.format(x_0))
                self.solve_times.append(1000*t_soln.to_sec())
                break
            elif self.perform_line_search:
                # Try again if expected solution time will not exceed replan loop allowance and have not exceeded max iteration count
                # or maximum allowable time factor for this replanning iteration
                have_time = ((((t_opt_finish-t_replan_start+t_soln).to_sec())*(1+self.t_replan_margin) < (1.0/self.replan_freq))
                                and i<(self.max_line_search_it-1) and self.f<f_max_curr)
                if have_time:
                    # Bad exit code. If we have time to do another solution increase time factor and try again
                    rospy.loginfo("Bad solver exit status: {}, {}. Trying again".format(str(prosta), str(solsta)))
                    rospy.loginfo("At t = {:.1f} s. f: {:.2f}".format(t_opt_finish.to_sec(),self.f))
                    # rospy.loginfo('x_0 = {}'.format(x_0))
                    continue
                else:
                    # Bad exit code and out of time. Keep flying off of last solution
                    rospy.loginfo("Line search ended in bad solver exit status: {}, {}".format(str(prosta), str(solsta)))
                    rospy.loginfo("At t = {:.1f} s.f: {:.2f}".format(t_opt_finish.to_sec(),self.f))
                    # rospy.loginfo('x_0 = {}'.format(x_0))
                    # self.task.writedata('failed_run.opf')
                    return
            else:
                # Bad exit code and not performing line search. Keep flying off of last solution
                rospy.loginfo("Bad solver exit status: {}, {}".format(str(prosta), str(solsta)))
                rospy.loginfo("At t = {:.1f} s".format(t_opt_finish.to_sec()))
                return

        # Store solution for use in output
        # Acquire lock to update trajectory
        self.trajectory_lock.acquire()
        self.task.getxx(mosek.soltype.itg,self.xx)
        self.cp_p_opt = self.xx[0:self.ind_vel]
        self.cp_v_opt = self.xx[self.ind_vel:self.ind_accel]
        self.cp_a_opt = self.xx[self.ind_accel:self.ind_jerk]
        self.cp_j_opt = self.xx[self.ind_jerk:self.ind_bin]
        self.bin_opt = self.xx[self.ind_bin:self.ind_bin + self.num_bin]
        self.dT_traj_opt = dT
        self.t_traj_opt_start = t_plan_start
        self.soln_no_opt = self.soln_no_opt + 1
        self.trajectory_lock.release() # Job's done!

        # Calculate a capability margin so that we back off when our current speed gets too close to constraints to
        # prevent infeasbility
        # TODO: This is a hack to get something working. The proper way is to make the constraints at the first interval
        # soft constraints and back off the time factor if the slack variables become active. Future work item possibly
        self.inst_capability_usage = np.max(np.hstack((np.abs(self.cp_v_opt[0:3*(self.n_cp-1)*self.n_int_capability_margin])/self.v_max,
                                                        np.abs(self.cp_a_opt[0:3*(self.n_cp-2)*self.n_int_capability_margin])/self.a_max,
                                                        np.abs(self.cp_j_opt[0:3*(self.n_cp-3)*self.n_int_capability_margin])/self.j_max)))

        # Fake planning option just runs from starting position
        if self.fake_plan:
            self.t_traj_opt_start = self.t_start_plan

        # Update path for visualization. Interpolate position values along entire length
        # TODO: Can probably clean the ordering on this one up. Reshape/swap axes works but is confusing
        p_interp,t_interp = bezier_curve(self.xx[0:self.ind_vel].reshape(self.n_cp,3,self.n_seg,order='F').swapaxes(0,1),
                                self.dT_traj_opt*np.ones(self.n_seg),num_disc = 20)
        iden_quat = Quaternion(x=0,y=0,z=0,w=1)
        new_pose_list = []
        new_path_header = Header(stamp=rospy.get_rostime(),frame_id = self.frame_id)
        for i in range(p_interp.shape[1]):
            new_pose_list.append(PoseStamped(new_path_header,Pose(position=Point(x=p_interp[0,i],y=p_interp[1,i],z=p_interp[2,i]),
                                                            orientation = iden_quat)))
        self.local_plan = Path(new_path_header,new_pose_list)

        # Store replan time for adjustments on future iterations
        t_replan_finish = rospy.get_rostime()
        self.replan_time_prev = (t_replan_finish-t_replan_start).to_sec()
        rospy.loginfo('Replanning Time: {:.1f} ms'.format(self.replan_time_prev*1000))

        # Have completed a replanning iteration, can start publishing goal location
        if not self.opt_run:
            # self.task.writedata('first_run.opf')
            self.opt_run = True
        
        # Replan successful!
        self.replan_successful = True

    def update_goal(self):

        if not self.opt_run:
            return

        # Interpolate local plan at current time
        t_curr = rospy.get_rostime()
        t_curr_s = t_curr.to_sec()

        state_start = copy.deepcopy(self.state)

        # See if we should update committed trajectory to latest optimal trajectory
        # TODO: Potentially need to address thread safety here, optimizer could be finishing as this runs
        # if t_curr_s>self.t_traj_opt_start:
        if self.soln_no_opt > self.soln_no_comm and t_curr_s>self.t_traj_opt_start:
            # Time to update committed trajectory, get lock
            self.trajectory_lock.acquire()
            self.cp_p_comm = self.cp_p_opt.copy()
            self.cp_v_comm = self.cp_v_opt.copy()
            self.cp_a_comm = self.cp_a_opt.copy()
            self.cp_j_comm = self.cp_j_opt.copy()
            self.bin_comm = self.bin_opt.copy()
            self.t_traj_comm_start = self.t_traj_opt_start
            self.dT_traj_comm = self.dT_traj_opt
            self.soln_no_comm = self.soln_no_opt
            self.trajectory_lock.release() # All done

        goal_p_interp = bezier_interpolate(self.cp_p_comm.reshape(self.n_cp,3,self.n_seg,order='F').swapaxes(0,1),
                                            self.dT_traj_comm*np.ones(self.n_seg),t_curr_s-self.t_traj_comm_start,n=3).flatten()
        goal_v_interp = bezier_interpolate(self.cp_v_comm.reshape(self.n_cp-1,3,self.n_seg,order='F').swapaxes(0,1),
                                            self.dT_traj_comm*np.ones(self.n_seg),t_curr_s-self.t_traj_comm_start,n=2).flatten()
        goal_a_interp = bezier_interpolate(self.cp_a_comm.reshape(self.n_cp-2,3,self.n_seg,order='F').swapaxes(0,1),
                                            self.dT_traj_comm*np.ones(self.n_seg),t_curr_s-self.t_traj_comm_start,n=1).flatten()
        goal_j_interp = bezier_interpolate(self.cp_j_comm.reshape(self.n_cp-3,3,self.n_seg,order='F').swapaxes(0,1),
                                            self.dT_traj_comm*np.ones(self.n_seg),t_curr_s-self.t_traj_comm_start,n=0).flatten()

        # Populate goal message
        goal_new = Goal()
        goal_new.p.x = goal_p_interp[0]
        goal_new.p.y = goal_p_interp[1]
        goal_new.p.z = goal_p_interp[2]
        goal_new.v.x = goal_v_interp[0]
        goal_new.v.y = goal_v_interp[1]
        goal_new.v.z = goal_v_interp[2]
        goal_new.a.x = goal_a_interp[0]
        goal_new.a.y = goal_a_interp[1]
        goal_new.a.z = goal_a_interp[2]
        goal_new.j.x = goal_j_interp[0]
        goal_new.j.y = goal_j_interp[1]
        goal_new.j.z = goal_j_interp[2]

        # Set yaw angle based on if we've reached or can see the goal
        if self.reached_goal:
            # Reached goal, clamp yaw value
            goal_new.yaw = self.yaw_filt_val
        elif not self.goal_in_view:
            # Rotate yaw angle towards goal
            heading_goal = np.arctan2(self.global_goal.pose.position.y-state_start.pos.y,self.global_goal.pose.position.x-state_start.pos.x)
            # quat_quad = [state_start.quat.x,state_start.quat.y,state_start.quat.z,state_start.quat.w]
            # euler_quad = euler_from_quaternion(quat_quad,'rzyx')
            delta_yaw = heading_goal - self.yaw_filt_val
            self.yaw_filt_val = self.yaw_filt_val + max(min(delta_yaw,self.yaw_change_find_goal_max),-self.yaw_change_find_goal_max)
            goal_new.yaw = self.yaw_filt_val
        else:
            # Align yaw with planned velocity direction
            # Slap a first order lag filter on it and clamp if speed is below a threshold
            if abs(goal_v_interp[0])<self.yaw_tol and abs(goal_v_interp[1])<self.yaw_tol:
                new_yaw_targ = self.yaw_filt_val
            else:
                new_yaw_targ = np.arctan2(goal_v_interp[1],goal_v_interp[0])
            yaw_filt = self.yaw_filt_coef*new_yaw_targ+(1-self.yaw_filt_coef)*self.yaw_filt_val
            self.yaw_filt_val = max(min(yaw_filt,self.yaw_filt_val+self.yaw_change_max),self.yaw_filt_val-self.yaw_change_max)
            goal_new.yaw = self.yaw_filt_val
        
        goal_new.header = Header(stamp=t_curr,frame_id = self.frame_id)

        if self.goal_log:
            if self.goal_log_count%self.goal_log_int == 0:
                rospy.loginfo("Published goal location [{:.3f},{:.3f},{:.3f}]".format(goal_new.p.x,goal_new.p.y,goal_new.p.z))
            self.goal_log_count += 1
        
        self.goal = goal_new

    def fake_dynamics(self):
        # Integrate jerk to fake having an IMU onboard
        if not self.opt_run:
            return

        # Interpolate jerk at current time and integrate with first order Euler
        t_curr = rospy.get_rostime()
        t_curr_s = t_curr.to_sec()
        
        jerk_interp = bezier_interpolate(self.cp_j_comm.reshape(self.n_cp-3,3,self.n_seg,order='F').swapaxes(0,1),
                                        self.dT_traj_comm*np.ones(self.n_seg),t_curr_s-self.t_traj_comm_start,n=0).flatten()

        self.fake_IMU = self.fake_IMU + jerk_interp/self.fake_dynamics_freq
    
    def streamprinter(self,text):
        # Define a stream printer to grab output from MOSEK
        sys.stdout.write(text)
        sys.stdout.flush()

    def replan_debug(self):
        # TODO: Should be able to delete these
        # Print interfaces to demonstrate they're coming in properly
        rospy.loginfo("Global Plan Position 1: %s",self.glob_plan.poses[0].pose.position)
        rospy.loginfo("Global Plan Position 2: %s",self.glob_plan.poses[1].pose.position)
        
        plane_1 = self.cvx_decomp.polyhedra[0].planes[0]
        plane_2 = self.cvx_decomp.polyhedra[0].planes[1]

        rospy.loginfo("Plane 1: %s",plane_1)
        rospy.loginfo("Plane 2: %s",plane_2)

    def update_goal_debug(self):
        # TODO: Should be able to delete these
        # Fudge the goal location
        curr_time = rospy.get_rostime()
        dt = curr_time-self.t_start_int_debug
        sine_freq = 0.5 # Hz
        sine_amp = 2 # m?
        self.goal.p.x = 0
        self.goal.p.y = 0
        self.goal.p.z = sine_amp*np.sin(2*pi*sine_freq*dt.to_sec())
        self.goal.a.y = 10*np.sin(2*2*pi*sine_freq*dt.to_sec())
        #rospy.loginfo("Applied z position %.3f m",self.goal.p.z)
        self.goal.header.frame_id = "map"
        self.goal.header.stamp = rospy.get_rostime()

        # Draw a sinusoidal path. S for snake
        new_path = Path()
        pose_list = []
        for i in range(6):
            made_up_pose = PoseStamped()
            made_up_pose.header.stamp = curr_time
            made_up_pose.header.frame_id = "map"
            made_up_pose.pose.position.x = 0.4*i
            made_up_pose.pose.position.y = ((0.4*i)**2)*np.sin(2*pi*sine_freq*dt.to_sec())
            made_up_pose.pose.position.z = 0
            made_up_pose.pose.orientation = Quaternion(x=0,y=0,z=0,w=1)
            pose_list.append(made_up_pose)
        new_path.poses = pose_list
        new_path.header.stamp = curr_time
        new_path.header.frame_id = "map"
        rospy.loginfo("Path has y position %.3f m",new_path.poses[5].pose.position.y)
        self.local_plan = new_path


if __name__ == '__main__':
    pass