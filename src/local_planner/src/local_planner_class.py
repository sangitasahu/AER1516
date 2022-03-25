#!/usr/bin/env python

#
# AER 1516 Motion Planning Project
# Local Planner
#

from __future__ import division, print_function, absolute_import
from cmath import pi

# Import libraries
import sys, copy
import rospy
import numpy as np
import mosek as mosek
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis, quaternion_multiply

# Helper functions
import bezier_curves

# Import message types
from nav_msgs.msg import Path
from snapstack_msgs.msg import State, Goal
from geometry_msgs.msg import Point, Vector3, Quaternion, PoseStamped
from shape_msgs.msg import Plane
from std_msgs.msg import Float64, Header
from local_planner.msg import CvxDecomp, Polyhedron

class LocalPlanner(object):
    "Local Planner Class"
    def __init__(self,replan_freq):
        
        # Storage
        # Inputs
        self.state = State()
        self.glob_plan = Path()
        self.cvx_decomp = CvxDecomp()
        self.global_goal = Point()

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

        # Planner parameters
        # Vehicle limits
        self.v_max = 3 # m/s
        self.a_max = 10 # m/s2
        self.j_max = 50 # m/s3

        # Path parameterization
        self.spline_deg = 3
        self.n_seg = 10
        self.n_cp = self.spline_deg+1

        # Optimizer parameters
        self.coll_M = 1000 # Collision constraint upper bound
        self.n_int_max = 5 # Maximum number of JPS intervals
        self.n_plane_max = 8 # Maximum number of polyhedron planes per interval
        self.log_settings = mosek.streamtype.log # Detail level of output
        self.set_var_names = True # Label variable names or not. Performance is allegedly faster without

        # Time line search parameters
        self.f = 3 # Factor on constant motion solution. Start conservatively
        self.replan_freq = replan_freq
        self.alpha = 1.25 # Replanning time factor to plan from on old solution
        self.gamma_up = 1 # Limit on how much time factor can increase/decrease on each timestep
        self.gamma_down = 0.25
        self.step_up = 0.5 # Step size for line search
        self.step_down = 0.25

        # Start MOSEK and initialize variables
        self.msk_env = mosek.Env()
        self.task = self.msk_env.Task()

        # Misc logging settings
        self.msk_env.set_Stream(mosek.streamtype.log, self.streamprinter) # Optimizer output
        self.task.set_Stream(mosek.streamtype.log, self.streamprinter) # Optimizer output
        self.task.putintparam(mosek.iparam.infeas_report_auto, mosek.onoffkey.on) # Infeasibility report

        # Storage for solution
        self.xx = np.zeros(self.numvar)
        self.cp_p_opt = np.zeros(self.num_pos)
        self.cp_v_opt = np.zeros(self.num_vel)
        self.cp_a_opt = np.zeros(self.num_accel)
        self.cp_j_opt = np.zeros(self.num_jerk)

        # Decision variables:
        # 4x position control points per segment,
        # 3x velocity, 2x acceleration, 1x jerk
        # 3x dimensions
        # n_int*n_seg binary variables
        self.num_pos = 3*self.n_seg*self.n_cp
        self.num_vel = 3*self.n_seg*(self.n_cp-1)
        self.num_accel = 3*self.n_seg*(self.n_cp-2)
        self.num_jerk = 3*self.n_seg*(self.n_cp-3)
        self.num_bin = self.n_int*self.n_seg

        self.numvar = self.num_pos+self.num_vel+self.num_accel+self.num_jerk+self.num_bin

        # Calculate indices of various locations for easy reference later
        self.ind_vel = self.num_pos
        self.ind_accel = self.ind_vel+self.num_vel
        self.ind_jerk = self.ind_accel+self.num_accel
        self.ind_bin = self.ind_jerk+self.num_jerk

        # Constraints:
        # Continuity, collision, consistency between p/v/a/j, binary variables
        self.num_cont = 3*self.spline_deg*(self.n_seg-1)
        self.num_planes = self.n_int_max*self.n_plane_max
        self.num_coll_per_seg = self.n_cp*self.num_planes
        self.num_coll = self.n_seg*self.num_coll_per_seg
        self.num_cons = int(round(self.n_seg*3*self.deg*(self.deg+1)/2))
        self.num_bin_sum = self.n_seg

        self.numcon = int(round(self.num_cont+self.num_coll+self.num_cons+self.num_bin_sum))

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

        # Add variables
        self.task.appendvars(self.numvar)
        self.task.appendcons(self.numcon)

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
                            ind_coll_this = self.ind_coll + self.n_plane_max*(k+self.n_int_max*(j+self.n_cp*i))
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
        a_sub_v_cons[0::3*(self.n_cp-1)] = ind_p0
        a_sub_v_cons[1::3*(self.n_cp-1)] = ind_p0+1
        a_sub_v_cons[2::3*(self.n_cp-1)] = ind_p0+1
        a_sub_v_cons[3::3*(self.n_cp-1)] = ind_p0+2
        a_sub_v_cons[4::3*(self.n_cp-1)] = ind_p0+2
        a_sub_v_cons[5::3*(self.n_cp-1)] = ind_p0+3
        
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
        a_sub_a_cons[0::3*(self.n_cp-2)] = ind_v0
        a_sub_a_cons[1::3*(self.n_cp-2)] = ind_v0+1
        a_sub_a_cons[2::3*(self.n_cp-2)] = ind_v0+1
        a_sub_a_cons[3::3*(self.n_cp-2)] = ind_v0+2
        
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
        a_sub_j_cons[0::3*(self.n_cp-3)] = ind_a0
        a_sub_j_cons[1::3*(self.n_cp-3)] = ind_a0+1
        
        ptrb_j_cons = np.arange(0,np.size(a_val_j_cons),3,dtype=np.int32)
        ptre_j_cons = np.arange(3,np.size(a_val_j_cons)+1,3,dtype=np.int32)
        
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
        ptrb_b_sum = np.arange(0,self.num_bin,self.n_int)
        ptre_b_sum = np.arange(self.n_int,self.num_bin+1,self.n_int)
        
        # Sum must be at least one
        self.blc[self.ind_bin_sum:(self.ind_bin_sum+self.n_seg)] = np.ones(self.n_seg)
        
        # Put coefficients for constraint
        self.task.putarowslice(self.ind_bin_sum,self.ind_bin_sum+self.num_bin_sum,ptrb_b_sum,ptre_b_sum,a_sub_b_sum,a_val_b_sum)
        
        # Set constraint bounds
        self.task.putconboundslice(0,self.numcon,self.bkc,self.blc,self.buc)

        # Cost function changes weakly with timestep. Just quadratic with dTs on the diagonal for jerk decision variables

        # Define objective function sense
        self.task.putobjsense(mosek.objsense.minimize)

        # Start time
        self.t_start = rospy.get_rostime()

    def __del__(self):
        # Close MOSEK
        self.task.__del__()
        self.msk_env.__del__()

    def replan(self):
        # Get start time for cutting off if we take too long
        t_replan_start = rospy.get_rostime()

        # REMINDER TO POTENTIALLY ADDRESS THREAD SAFETY
        state_start = copy.deepcopy(self.state)
        global_plan = copy.deepcopy(self.glob_plan)
        cvx_decomp = copy.deepcopy(self.cvx_decomp)

        # Problem size
        n_int = len(cvx_decomp.polyhedra)
        n_int_glob = len(global_plan.poses)
        if not(n_int==n_int_glob):
            rospy.loginfo("Number of polyhedra does not match global path length({}~={})".format(n_int,n_int_glob))
        n_planes = np.zeros(n_int)
        for i in range(n_int):
            n_planes[i] = len(cvx_decomp.polyhedra[i])
        
        # Calculate time allocation
        # Calculate minimum time for constant input motions between current and goal states, multiply by adaptive scale factor
        x_0 = np.array([state_start.pos.x,state_start.pos.y,state_start.pos.z])
        v_0 = np.array([state_start.vel.x,state_start.vel.y,state_start.vel.z])
        x_f = np.array([global_plan.poses[n_int].pose.position.x,
                        global_plan.poses[n_int].pose.position.y,
                        global_plan.poses[n_int].pose.position.z])
        v_f = np.zeros(3)
        a_f = np.zeros(3)
        
        dx = x_f-x_0
        dv = v_f-v_0
        t_min = np.max(np.vstack((np.abs(dx)/self.v_max,
                                np.abs(dv)/self.a_max)))
        
        t_alloc = t_min*self.f
        dT = t_alloc/self.n_seg

        # Update bounds and constraints based on parameters for this timestep
        # ICs/BCs via variable bounds
        # Velocity/position at initial condition, p/v/a at final condition (acceleration not defined in fake sim)
        bkx_ic_bc = [mosek.boundkey.fx]*3*5
        blx_ic_bc = np.hstack((x_0,v_0,x_f,v_f,a_f))
        bux_ic_bc = blx_ic_bc
        b_ind_ic_bc = []

        ic_all = np.vstack((x_0,v_0))
        bc_all = np.vstack((x_f,v_f,a_f))
        # Initial conditions
        ind_table = [0,self.ind_vel]
        for i in range(2):
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

        # Velocity/Acceleration/Jerk Consistency - Just need to update
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

        # Collision Inequality
        # Extract plane coefficients from convex decomposition
        plane_norms = np.zeros((self.n_int_max,self.n_plane_max,3))
        plane_coefs = np.zeros((self.n_int_max,self.n_plane_max))
        for i in range(len(cvx_decomp.polyhedra)):
            for j in range(len(cvx_decomp.polyhedra[i].planes)):
                plane_norms[i,j,0] = cvx_decomp.polyhedra[i].planes[j].a
                plane_norms[i,j,1] = cvx_decomp.polyhedra[i].planes[j].b
                plane_norms[i,j,2] = cvx_decomp.polyhedra[i].planes[j].c
                plane_coefs[i,j] = cvx_decomp.polyhedra[i].planes[j].d
        coll_rel_row_inds = np.arange(self.n_plane_max*self.n_int_max).reshape(plane_coefs.shape)


        for i in range(self.n_seg):
            # Loop over curve segments. All control points in each segment must be checked against all planes
            # along with binary variables to indicate which constraints are active                                 
            ind_coll_start = ind_coll+i*num_coll_per_seg
            ind_coll_end = ind_coll_start + num_coll_per_seg
            
            # Index row coefficients by running down each column in turn. Requires 4 passes for all coefficients
            coll_subi = np.matmul(np.arange(ind_coll_start,ind_coll_end).reshape((num_coll_per_seg,1)),
                            np.ones((1,4),dtype=np.int32)).flatten(order='F')
            # Column indices need to be repeated num_planes times
            coll_subj_cps = np.matmul(np.arange(i*3*n_cp,i*3*n_cp+3*n_cp).reshape((3*n_cp,1)),
                                np.ones((1,num_planes),dtype=np.int32)).flatten(order='C')
            # Column indices of binary coefficients. Stacking order has them along diagonals for each control point
            coll_subj_bin_temp = (np.tile(np.arange(ind_bin+i*n_int,ind_bin+(i+1)*n_int),plane_coefs.shape[1])
                                .reshape((plane_coefs.shape[1],n_int),order='C'))
            coll_subj_bin = np.tile(coll_subj_bin_temp.flatten(order='F'),n_cp)
            
            # Actual coefficients of constraint matrix are based on plane normals
            coll_a_x = np.tile(plane_norms[:,:,0].flatten(order='C'),n_cp)
            coll_a_y = np.tile(plane_norms[:,:,1].flatten(order='C'),n_cp)
            coll_a_z = np.tile(plane_norms[:,:,2].flatten(order='C'),n_cp)
            
            # Binary variables just get the M coefficient
            coll_a_bin = -M*np.ones(coll_subj_bin.shape[0])
            
            # Put the A matrix and RHS
            # RHS is simply plane coefficients repeated for each control point plus the M coefficient
            task.putaijlist(coll_subi,np.hstack((coll_subj_cps,coll_subj_bin)),
                            np.hstack((coll_a_x,coll_a_y,coll_a_z,coll_a_bin)))
            blc[ind_coll_start:ind_coll_end] = np.tile(plane_coefs.flatten(order='C'),n_cp)-M

            self.task.putarowslice(self.ind_coll,self.ind_bin,ptrb,ptre,a_sub_j,a_val_coll)
        
        # Update fix inactive binary variables

        potato = 5

    def update_goal(self):
        potato = 5

    def streamprinter(text):
        # Define a stream printer to grab output from MOSEK
        sys.stdout.write(text)
        sys.stdout.flush()

    def replan_debug(self):
        # Print interfaces to demonstrate they're coming in properly
        rospy.loginfo("Global Plan Position 1: %s",self.glob_plan.poses[0].pose.position)
        rospy.loginfo("Global Plan Position 2: %s",self.glob_plan.poses[1].pose.position)
        
        plane_1 = self.cvx_decomp.polyhedra[0].planes[0]
        plane_2 = self.cvx_decomp.polyhedra[0].planes[1]

        rospy.loginfo("Plane 1: %s",plane_1)
        rospy.loginfo("Plane 2: %s",plane_2)

    def update_goal_debug(self):
        # Fudge the goal location
        curr_time = rospy.get_rostime()
        dt = curr_time-self.t_start
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
            pose_list.append(made_up_pose)
        new_path.poses = pose_list
        new_path.header.stamp = curr_time
        new_path.header.frame_id = "map"
        rospy.loginfo("Path has y position %.3f m",new_path.poses[5].pose.position.y)
        self.local_plan = new_path


if __name__ == '__main__':
    pass