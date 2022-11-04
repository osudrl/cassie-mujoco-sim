# Copyright (c) 2018 Dynamic Robotics Laboratory
#
# Permission to use, copy, modify, and distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

from cassiemujoco_ctypes import *
import os
import ctypes
import numpy as np

# from envs.terrains.rand import *

# Get base directory
_dir_path = os.path.dirname(os.path.realpath(__file__))

# Initialize libcassiesim
default_model = "../model/cassie.xml"
cassie_mujoco_init(str.encode(default_model))

# Interface classes
# Note: Making the optional argument be a global var be default is perhaps not the safest thing to do
class CassieSim:
    def __init__(self, modelfile=default_model, terrain=False, perception=False, reinit=False):

        if modelfile is not default_model:
            self.modelfile = modelfile
        else:
            base = 'cassie'
            if perception:
                base += '_perception'
            if terrain:
                base += '_hfield'
            self.modelfile = os.path.join(_dir_path, base + '.xml')

        self.c = cassie_sim_init(self.modelfile.encode('utf-8'), True)

        if terrain:
            x_res, y_res = self.get_hfield_nrow(), self.get_hfield_ncol()
            self.hfields = generate_perlin(x_res, y_res)

        params_array = (ctypes.c_int32 * 6)()
        cassie_sim_params(self.c, params_array)
        self.nv = cassie_sim_nv(self.c)
        self.nbody = cassie_sim_nbody(self.c)
        self.nq = cassie_sim_nq(self.c)
        self.ngeom = cassie_sim_ngeom(self.c)

    def randomize_terrain(self):
        hfield = self.hfields[np.random.randint(len(self.hfields))]
        self.set_hfield_data(hfield.flatten())
        return hfield

    def step(self, u):
        y = cassie_out_t()
        cassie_sim_step(self.c, y, u)
        return y

    def step_pd(self, u):
        y = state_out_t()
        cassie_sim_step_pd(self.c, y, u)
        return y

    def integrate_pos(self):
        y = state_out_t()
        cassie_integrate_pos(self.c, y)
        return y

    def get_state(self):
        s = CassieState()
        cassie_get_state(self.c, s.s)
        return s

    def set_state(self, s):
        cassie_set_state(self.c, s.s)

    def time(self):
        timep = cassie_sim_time(self.c)
        return timep[0]

    def qpos(self):
        qposp = cassie_sim_qpos(self.c)
        return qposp[:self.nq]

    def qpos_full(self):
        qposp = cassie_sim_qpos(self.c)
        return qposp[:self.nq]

    def qvel(self):
        qvelp = cassie_sim_qvel(self.c)
        return qvelp[:self.nv]

    def qvel_full(self):
        qvelp = cassie_sim_qvel(self.c)
        return qvelp[:self.nv]

    def qacc(self):
        qaccp = cassie_sim_qacc(self.c)
        return qaccp[:self.nv]

    def xpos(self, body_name):
        xposp = cassie_sim_xpos(self.c, body_name.encode())
        return xposp[:3]

    def xquat(self, body_name):
        xquatp = cassie_sim_xquat(self.c, body_name.encode())
        return xquatp[:4]

    def set_time(self, time):
        timep = cassie_sim_time(self.c)
        timep[0] = time

    def set_qpos(self, qpos):
        qposp = cassie_sim_qpos(self.c)
        for i in range(min(len(qpos), self.nq)):
            qposp[i] = qpos[i]

    def set_qvel(self, qvel):
        qvelp = cassie_sim_qvel(self.c)
        for i in range(min(len(qvel), self.nv)):
            qvelp[i] = qvel[i]

    def hold(self):
        cassie_sim_hold(self.c)

    def release(self):
        cassie_sim_release(self.c)

    def apply_force(self, xfrc, body_name="cassie-pelvis"):
        xfrc_array = (ctypes.c_double * 6)()
        for i in range(len(xfrc)):
            xfrc_array[i] = xfrc[i]
        cassie_sim_apply_force(self.c, xfrc_array, body_name.encode())

    def sense_ground(self):
        percept = (ctypes.c_double * 6)()
        cassie_sim_read_rangefinder(self.c, percept)

        ret = np.zeros(6)
        for i in range(6):
            ret[i] = percept[i]
        return ret

    def get_jacobian(self, name):
        jacp = np.zeros(3*self.nv)
        jacp_array = (ctypes.c_double * (3*self.nv))()
        cassie_sim_get_jacobian(self.c, jacp_array, name.encode())
        for i in range(3*self.nv):
            jacp[i] = jacp_array[i]
        return jacp

    def get_jacobian_full(self, name):
        jacp = np.zeros(3*self.nv)
        jacp_array = (ctypes.c_double * (3*self.nv))()
        jacr = np.zeros(3*self.nv)
        jacr_array = (ctypes.c_double * (3*self.nv))()
        cassie_sim_get_jacobian_full(self.c, jacp_array, jacr_array, name.encode())
        for i in range(3*self.nv):
            jacp[i] = jacp_array[i]
            jacr[i] = jacr_array[i]
        return jacp, jacr

    def get_jacobian_full_site(self, name):
        jacp = np.zeros(3*self.nv)
        jacp_array = (ctypes.c_double * (3*self.nv))()
        jacr = np.zeros(3*self.nv)
        jacr_array = (ctypes.c_double * (3*self.nv))()
        cassie_sim_get_jacobian_full_site(self.c, jacp_array, jacr_array, name.encode())
        for i in range(3*self.nv):
            jacp[i] = jacp_array[i]
            jacr[i] = jacr_array[i]
        return jacp, jacr

    def get_foot_forces(self):
        force = np.zeros(12)
        frc_array = (ctypes.c_double * 12)()
        cassie_sim_foot_forces(self.c, frc_array)
        for i in range(12):
            force[i] = frc_array[i]
        lfrc = np.sqrt(np.power(force[0:3], 2).sum())
        rfrc = np.sqrt(np.power(force[6:9], 2).sum())
        return lfrc, rfrc

    # Returns 2 arrays each 6 long, the toe force and heel force. Each array is in order of 
    # left foot (3) and then right foot (3)
    def get_heeltoe_forces(self):
        toe_force = np.zeros(6)
        heel_force = np.zeros(6)
        toe_array = (ctypes.c_double * 6)()
        heel_array = (ctypes.c_double * 6)()
        cassie_sim_heeltoe_forces(self.c, toe_array, heel_array)
        for i in range(6):
            toe_force[i] = toe_array[i]
            heel_force[i] = heel_array[i]
        return toe_force, heel_force

    def foot_pos(self):
        pos_array = (ctypes.c_double * 6)()
        cassie_sim_foot_positions(self.c, pos_array)
        pos = []
        for i in range(6):
            pos.append(pos_array[i])
        return pos

    def foot_vel(self, vel):
        vel_array = (ctypes.c_double * 12)()
        cassie_sim_foot_velocities(self.c, vel_array)
        for i in range(12):
            vel[i] = vel_array[i]

    def body_vel(self, vel, body_name):
        vel_array = (ctypes.c_double * 6)()
        cassie_sim_body_vel(self.c, vel_array, body_name.encode())
        for i in range(6):
            vel[i] = vel_array[i]

    # Returns the center of mass position vector in world frame
    def center_of_mass_position(self):
        pos_array = (ctypes.c_double * 3)()
        cassie_sim_cm_position(self.c, pos_array)
        pos = []
        for i in range(3):
            pos.append(pos_array[i])
        return pos

    # Returns the center of mass velocity vector in world frame
    def center_of_mass_velocity(self):
        vel_array = (ctypes.c_double * 3)()
        cassie_sim_cm_velocity(self.c, vel_array)
        vel = []
        for i in range(3):
            vel.append(vel_array[i])
        return vel

    # Returns 3x3 rotational intertia matrix of the robot around its center
    # of mass in the pelvis frame. [kg*m^2]
    def centroid_inertia(self):
        I_array = (ctypes.c_double * 9)()
        cassie_sim_centroid_inertia(self.c, I_array)
        Icm = []
        for i in range(9):
            Icm.append(I_array[i])
        return Icm

    # Return the angular momentum of the robot in the world frame.
    def angular_momentum(self):
        L_array = (ctypes.c_double * 3)()
        cassie_sim_angular_momentum(self.c, L_array)
        L_return = []
        for i in range(3):
            L_return.append(L_array[i])
        return L_return

    # Return the full 32x32 mass matrix of Cassie.
    def full_mass_matrix(self):
        M_array = (ctypes.c_double * (32 * 32))()
        cassie_sim_full_mass_matrix(self.c, M_array)
        M_return = np.zeros((32,32))
        for i in range(32):
            for j in range(32):
                M_return[i,j] = M_array[i*32+j]
        return M_return

    def constraint_jacobian(self):
        J_array = (ctypes.c_double * (6 * 32))()
        err_array = (ctypes.c_double * (6))()
        cassie_sim_loop_constraint_info(self.c, J_array, err_array)
        J_return = np.zeros((6,32))
        for i in range(6):
            for j in range(32):
                J_return[i,j] = J_array[i*32+j]
        return J_return

    def constraint_error(self):
        J_array = (ctypes.c_double * (6 * 32))()
        err_array = (ctypes.c_double * (6))()
        cassie_sim_loop_constraint_info(self.c, J_array, err_array)
        err_return = np.zeros((6,1))
        for i in range(6):
                err_return[i] = err_array[i]
        return err_return

    # Return the minimal actuated mass matrix of Cassie. Contains 6 for floating 
    # base, 5 for left leg motors, 5 for right leg motors.
    def minimal_mass_matrix(self):
        ind_full_idx = [0,1,2,3,4,5,6,7,8,12,18,19,20,21,25,31]
        dep_full_idx = [9,10,11,14,22,23,24,27]
        ind_idx = np.arange(0, 16)
        dep_idx = np.arange(16, 24)
        spring_idx = [13, 15, 26, 28]

        J_c = self.constraint_jacobian()
        J_c[:, 0:6] = 0  # Zero out floating base coordinates
        J_c_div = J_c[:, ind_full_idx + dep_full_idx]
        J_c_div[J_c_div < 1e-5] = 0 # Zero out very small terms for numerical stabilityL 
        
        M = self.full_mass_matrix()
        M_div_temp = M[:, ind_full_idx + dep_full_idx]
        M_div = M_div_temp[ind_full_idx + dep_full_idx, :]

        G = np.linalg.lstsq(J_c_div[:, dep_idx], -J_c_div[:, ind_idx])

        # print(G[0].shape)
        P = np.block([[np.eye(16)], [G[0]]])
        M_minimal = P.T  @ M_div @ P

        #J_c_dep = J_c[:, (13,24)]
        # This gets hard 
        # import sys
        # np.set_printoptions(threshold=sys.maxsize)
        M_print = M_div[:, ind_idx]
        M_print = M_print[ind_idx, :]

        # print("M")
        # print(M_print)
        # print("M_minimal")
        # print(M_minimal)


        return M_minimal
        

    def foot_quat(self, quat):
        quat_array = (ctypes.c_double * 4)()
        cassie_sim_foot_quat(self.c, quat_array)
        for i in range(4):
            quat[i] = quat_array[i]

    def clear_forces(self):
        cassie_sim_clear_forces(self.c)

    def get_dof_damping(self):
        ptr = cassie_sim_dof_damping(self.c)
        ret = np.zeros(self.nv)
        for i in range(self.nv):
            ret[i] = ptr[i]
        return ret
    
    def get_body_mass(self):
        ptr = cassie_sim_body_mass(self.c)
        ret = np.zeros(self.nbody)
        for i in range(self.nbody):
            ret[i] = ptr[i]
        return ret

    def get_body_ipos(self):
        nbody = self.nbody * 3
        ptr = cassie_sim_body_ipos(self.c)
        ret = np.zeros(nbody)
        for i in range(nbody):
            ret[i] = ptr[i]
        return ret

    def get_body_pos(self, name):
        ptr = cassie_sim_get_body_name_pos(self.c, name.encode())
        ret = np.zeros(3)
        for i in range(3):
            ret[i] = ptr[i]
        return ret

    def get_geom_friction(self):
        ptr = cassie_sim_geom_friction(self.c)
        ret = np.zeros(self.ngeom * 3)
        for i in range(self.ngeom * 3):
            ret[i] = ptr[i]
        return ret

    def get_geom_rgba(self, name=None):
        if name is not None:
            ptr = cassie_sim_geom_name_rgba(self.c, name.encode())
            ret = np.zeros(4)
            for i in range(4):
                ret[i] = ptr[i]
        else:
            ptr = cassie_sim_geom_rgba(self.c)
            ret = np.zeros(self.ngeom * 4)
            for i in range(self.ngeom * 4):
                ret[i] = ptr[i]
        return ret

    def get_geom_quat(self, name=None):
        if name is not None:
            ptr = cassie_sim_geom_name_quat(self.c, name.encode())
            ret = np.zeros(4)
            for i in range(4):
                ret[i] = ptr[i]
        else:
            ptr = cassie_sim_geom_quat(self.c)
            ret = np.zeros(self.ngeom * 4)
            for i in range(self.ngeom * 4):
                ret[i] = ptr[i]
        return ret

    def get_geom_pos(self, name=None):
        if name is not None:
            ptr = cassie_sim_geom_name_pos(self.c, name.encode())
            ret = np.zeros(3)
            for i in range(3):
                ret[i] = ptr[i]
        else:
            ptr = cassie_sim_geom_pos(self.c)
            ret = np.zeros(self.ngeom * 3)
            for i in range(self.ngeom * 3):
                ret[i] = ptr[i]
        return ret
    
    def get_geom_size(self, name=None):
        if name is not None:
            ptr = cassie_sim_geom_name_size(self.c, name.encode())
            ret = np.zeros(3)
            for i in range(3):
                ret[i] = ptr[i]
        else:
            ptr = cassie_sim_geom_size(self.c)
            ret = np.zeros(self.ngeom * 3)
            for i in range(self.ngeom * 3):
                ret[i] = ptr[i]
        return ret

    def set_dof_damping(self, data):
        c_arr = (ctypes.c_double * self.nv)()

        if len(data) != self.nv:
            print("SIZE MISMATCH SET_DOF_DAMPING()")
            exit(1)
        
        for i in range(self.nv):
            c_arr[i] = data[i]

        cassie_sim_set_dof_damping(self.c, c_arr)

    def set_body_mass(self, data, name=None):
        # If no name is provided, set ALL body masses and assume "data" is array
        # containing masses for every body
        if name is None:
            c_arr = (ctypes.c_double * self.nbody)()

            if len(data) != self.nbody:
                print("SIZE MISMATCH SET_BODY_MASS()")
                exit(1)
            
            for i in range(self.nbody):
                c_arr[i] = data[i]

            cassie_sim_set_body_mass(self.c, c_arr)
        # If name is provided, only set mass for specified body and assume
        # "data" is a single double
        else:
            cassie_sim_set_body_name_mass(self.c, name.encode(), ctypes.c_double(data))

    def set_body_pos(self, name, data):
        if len(data) != 3:
            print("SIZE MISMATCH SET BODY POS") 
            exit(1)
        c_arr = (ctypes.c_double * 3)()
        for i in range(3):
            c_arr[i] = data[i]
        cassie_sim_set_body_name_pos(self.c, name.encode(), c_arr)

    def set_body_ipos(self, data):
        nbody = self.nbody * 3
        c_arr = (ctypes.c_double * nbody)()

        if len(data) != nbody:
            print("SIZE MISMATCH SET_BODY_IPOS()")
            exit(1)
        
        for i in range(nbody):
            c_arr[i] = data[i]

        cassie_sim_set_body_ipos(self.c, c_arr)

    def set_geom_friction(self, data, name=None):
        if name is None:
            c_arr = (ctypes.c_double * (self.ngeom*3))()

            if len(data) != self.ngeom*3:
                print("SIZE MISMATCH SET_GEOM_FRICTION()")
                exit(1)

            for i in range(self.ngeom*3):
                c_arr[i] = data[i]

            cassie_sim_set_geom_friction(self.c, c_arr)
        else:
            fric_array = (ctypes.c_double * 3)()
            for i in range(3):
                fric_array[i] = data[i]
            cassie_sim_set_geom_name_friction(self.c, name.encode(), fric_array)


    def set_geom_rgba(self, data, name=None):
        if name is None:
                ngeom = self.ngeom * 4

                if len(data) != ngeom:
                    print("SIZE MISMATCH SET_GEOM_RGBA()")
                    exit(1)

                c_arr = (ctypes.c_float * ngeom)()

                for i in range(ngeom):
                    c_arr[i] = data[i]

                cassie_sim_set_geom_rgba(self.c, c_arr)
        else:
            rgba_array = (ctypes.c_float * 4)()
            for i in range(4):
                rgba_array[i] = data[i]
            cassie_sim_set_geom_name_rgba(self.c, name.encode(), rgba_array)
    
    def set_geom_quat(self, data, name=None):
        if name is None:
            ngeom = self.ngeom * 4

            if len(data) != ngeom:
                print("SIZE MISMATCH SET_GEOM_QUAT()")
                exit(1)

            c_arr = (ctypes.c_double * ngeom)()

            for i in range(ngeom):
                c_arr[i] = data[i]

            cassie_sim_set_geom_quat(self.c, c_arr)
        else:
            quat_array = (ctypes.c_double * 4)()
            for i in range(4):
                quat_array[i] = data[i]
            cassie_sim_set_geom_name_quat(self.c, name.encode(), quat_array)

    
    def set_geom_pos(self, data, name=None):
        if name is None:
            ngeom = self.ngeom * 3

            if len(data) != ngeom:
                print("SIZE MISMATCH SET_GEOM_POS()")
                exit(1)

            c_arr = (ctypes.c_double * ngeom)()

            for i in range(ngeom):
                c_arr[i] = data[i]

            cassie_sim_set_geom_pos(self.c, c_arr)
        else:
            array = (ctypes.c_double * 3)()
            for i in range(3):
                array[i] = data[i]
            cassie_sim_set_geom_name_pos(self.c, name.encode(), array)

    def set_geom_size(self, data, name=None):
        if name is None:
            ngeom = self.ngeom * 3

            if len(data) != ngeom:
                print("SIZE MISMATCH SET_GEOM_POS()")
                exit(1)

            c_arr = (ctypes.c_double * ngeom)()

            for i in range(ngeom):
                c_arr[i] = data[i]

            cassie_sim_set_geom_size(self.c, c_arr)
        else:
            array = (ctypes.c_double * 3)()
            for i in range(3):
                array[i] = data[i]
            cassie_sim_set_geom_name_size(self.c, name.encode(), array)

    def set_const(self):
        cassie_sim_set_const(self.c)

    def full_reset(self):
        cassie_sim_full_reset(self.c)

    def get_hfield_nrow(self):
        return cassie_sim_get_hfield_nrow(self.c)

    def get_hfield_ncol(self):
        return cassie_sim_get_hfield_ncol(self.c)

    def get_nhfielddata(self):
        return cassie_sim_get_nhfielddata(self.c)

    def get_hfield_size(self):
        ret = np.zeros(4)
        ptr = cassie_sim_get_hfield_size(self.c)
        for i in range(4):
            ret[i] = ptr[i]
        return ret

    # Note that data has to be a flattened array. If flattening 2d numpy array, rows are y axis
    # and cols are x axis. The data must also be normalized to (0-1)
    def set_hfield_data(self, data, vis=None):
        nhfielddata = self.get_nhfielddata()
        if len(data) != nhfielddata:
            print("SIZE MISMATCH SET_HFIELD_DATA")
            exit(1)
        data_arr = (ctypes.c_float * nhfielddata)(*data)
        cassie_sim_set_hfielddata(self.c, ctypes.cast(data_arr, ctypes.POINTER(ctypes.c_float)))

        if vis is not None:
            cassie_vis_remakeSceneCon(vis)
    
    def get_hfield_data(self):
        nhfielddata = self.get_nhfielddata()
        ret = np.zeros(nhfielddata)
        ptr = cassie_sim_hfielddata(self.c)
        for i in range(nhfielddata):
            ret[i] = ptr[i]
        return ret

    def set_hfield_size(self, data):
        if len(data) != 4:
            print("SIZE MISMATCH SET_HFIELD_SIZE")
            exit(1)
        size_array = (ctypes.c_double * 4)()
        for i in range(4):
            size_array[i] = data[i]
        cassie_sim_set_hfield_size(self.c, size_array)

    # Returns a pointer to an array of joint_filter_t objects. Can be accessed/indexed as a usual python array of
    # joint filter objects
    def get_joint_filter(self):
        j_filters = cassie_sim_joint_filter(self.c)
        return j_filters

    # Set interal state of the joint filters. Takes in 2 arrays of values (x and y), which should be 6*4 and 6*3 long respectively. 
    # (6 joints, for each joint x has 4 values y has 3)
    def set_joint_filter(self, x, y):
        x_arr = (ctypes.c_double * (6*4))(*x)
        y_arr = (ctypes.c_double * (6*3))(*y)
        cassie_sim_set_joint_filter(self.c, ctypes.cast(x_arr, ctypes.POINTER(ctypes.c_double)), ctypes.cast(y_arr, ctypes.POINTER(ctypes.c_double)))

    # Returns a pointer to an array of drive_filter_t objects. Can be accessed/indexed as a usual python array of
    # drive filter objects
    def get_drive_filter(self):
        d_filters = cassie_sim_drive_filter(self.c)
        return d_filters

    # Set interal state of the drive filters. Takes in an array of values (x), which should be 10*9 long.
    # (10 motor, for each motor have 9 values)
    def set_drive_filter(self, x):
        x_arr = (ctypes.c_int * (10*9))(*x)
        cassie_sim_set_drive_filter(self.c, ctypes.cast(x_arr, ctypes.POINTER(ctypes.c_int)))

    # Get the current state of the torque delay array. Returns a 2d numpy array of size (10, 6), 
    # number of motors by number of delay cycles
    def get_torque_delay(self):
        t_arr = (ctypes.c_double * 60)()
        cassie_sim_torque_delay(self.c, t_arr)
        return np.array(t_arr[:]).reshape((10, 6))

    # Set the torque delay state. Takes in a 2d numpy array of size (10, 6), number of motors by number of delay cycles
    def set_torque_delay(self, data):
        set_t_arr = (ctypes.c_double * 60)(*data.flatten())
        cassie_sim_set_torque_delay(self.c, ctypes.cast(set_t_arr, ctypes.POINTER(ctypes.c_double)))

    def __del__(self):
        cassie_sim_free(self.c)

class CassieVis:
    def __init__(self, c, offscreen=False):
        self.v = cassie_vis_init(c.c, c.modelfile.encode('utf-8'), ctypes.c_bool(offscreen))
        self.is_recording = False

    def draw(self, c):
        state = cassie_vis_draw(self.v, c.c)
        return state

    def get_extent(self):
        return cassie_vis_extent(self.v)

    def get_znear(self):
        return cassie_vis_znear(self.v)

    def get_zfar(self):
        return cassie_vis_zfar(self.v)

    def valid(self):
        return cassie_vis_valid(self.v)

    def ispaused(self):
        return cassie_vis_paused(self.v)

    def remake(self):
        cassie_vis_remakeSceneCon(self.v)

    # Applies the inputted force to the inputted body. "xfrc_apply" should contain the force/torque to 
    # apply in Cartesian coords as a 6-long array (first 3 are force, last 3 are torque). "body_name" 
    # should be a string matching a body name in the XML file. If "body_name" doesn't match an existing
    # body name, then no force will be applied. 
    def apply_force(self, xfrc_apply, body_name):
        xfrc_array = (ctypes.c_double * 6)()
        for i in range(len(xfrc_apply)):
            xfrc_array[i] = xfrc_apply[i]
        cassie_vis_apply_force(self.v, xfrc_array, body_name.encode())

    def add_marker(self, pos, size, rgba, so3):
        pos_array = (ctypes.c_double * 3)()
        for i in range(len(pos)):
            pos_array[i] = pos[i]
        size_array = (ctypes.c_double * 3)()
        for i in range(len(size)):
            size_array[i] = size[i]
        rgba_array = (ctypes.c_double * 4)()
        for i in range(len(rgba)):
            rgba_array[i] = rgba[i]
        so3_array = (ctypes.c_double * 9)()
        for i in range(len(so3)):
            so3_array[i] = so3[i]
        cassie_vis_add_marker(self.v, pos_array, size_array, rgba_array, so3_array)

    def remove_marker(self, id_val):
        cassie_vis_remove_marker(self.v, id_val)

    def clear_markers(self):
        cassie_vis_clear_markers(self.v)

    def update_marker(self, id_val, pos, size, rgba, so3):
        pos_array = (ctypes.c_double * 3)()
        size_array = (ctypes.c_double * 3)()
        rgba_array = (ctypes.c_double * 4)()
        so3_array = (ctypes.c_double * 9)()
        for i in range(len(pos)):
            pos_array[i] = pos[i]
        for i in range(len(size)):
            size_array[i] = size[i]
        for i in range(len(rgba)):
            rgba_array[i] = rgba[i]
        for i in range(len(so3)):
            so3_array[i] = so3[i]
        cassie_vis_update_marker_pos(self.v, ctypes.c_int(id_val), pos_array)
        cassie_vis_update_marker_size(self.v, ctypes.c_int(id_val), size_array)
        cassie_vis_update_marker_rgba(self.v, ctypes.c_int(id_val), rgba_array)
        cassie_vis_update_marker_orient(self.v, ctypes.c_int(id_val), so3_array)

    def reset(self, c):
        cassie_vis_full_reset(self.v, c.c)
        #cassie_vis_close(self.v)
        #cassie_vis_free(self.v)
        #delattr(self, 'v')
        #self.v = cassie_vis_init(c.c, c.modelfile.encode('utf-8'))

    def set_cam(self, body_name, zoom, azimuth, elevation):
        cassie_vis_set_cam(self.v, body_name.encode(), zoom, azimuth, elevation)

    def window_resize(self, width=1200, height=900):
        cassie_vis_window_resize(self.v, ctypes.c_int(width), ctypes.c_int(height))

    def attach_cam(self, cam_name='egocentric'):
        cassie_vis_attach_cam(self.v, cam_name.encode())

    def init_depth(self, width, height):
        cassie_vis_init_depth(self.v, ctypes.c_int(width), ctypes.c_int(height))

    def get_depth_size(self):
        size = cassie_vis_get_depth_size(self.v)
        return size

    def draw_depth(self, c, width=30, height=30):
        depth = cassie_vis_draw_depth(self.v, c.c, ctypes.c_int(width), ctypes.c_int(height))
        return depth[:width*height]

    def init_recording(self, filename, width=1920, height=1080):
        cassie_vis_init_recording(self.v, filename.encode(), ctypes.c_int(width), ctypes.c_int(height))
        self.is_recording = True

    def record_frame(self):
        cassie_vis_record_frame(self.v)

    def close_recording(self):
        cassie_vis_close_recording(self.v)
        self.is_recording = False

    def __del__(self):
        cassie_vis_free(self.v)

class CassieUdp:
    def __init__(self, remote_addr='127.0.0.1', remote_port='25000',
                 local_addr='0.0.0.0', local_port='25001'):
        self.sock = udp_init_client(str.encode(remote_addr),
                                    str.encode(remote_port),
                                    str.encode(local_addr),
                                    str.encode(local_port))
        self.packet_header_info = packet_header_info_t()
        self.recvlen = 2 + 697
        self.sendlen = 2 + 58
        self.recvlen_pd = 2 + 493
        self.sendlen_pd = 2 + 476
        self.recvbuf = (ctypes.c_ubyte * max(self.recvlen, self.recvlen_pd))()
        self.sendbuf = (ctypes.c_ubyte * max(self.sendlen, self.sendlen_pd))()
        self.inbuf = ctypes.cast(ctypes.byref(self.recvbuf, 2),
                                 ctypes.POINTER(ctypes.c_ubyte))
        self.outbuf = ctypes.cast(ctypes.byref(self.sendbuf, 2),
                                    ctypes.POINTER(ctypes.c_ubyte))

    def send(self, u):
        pack_cassie_user_in_t(u, self.outbuf)
        send_packet(self.sock, self.sendbuf, self.sendlen, None, 0)

    def send_pd(self, u):
        pack_pd_in_t(u, self.outbuf)
        send_packet(self.sock, self.sendbuf, self.sendlen_pd, None, 0)

    def recv_wait(self):
        nbytes = -1
        while nbytes != self.recvlen:
            nbytes = get_newest_packet(self.sock, self.recvbuf, self.recvlen,
                                         None, None)
        process_packet_header(self.packet_header_info,
                                self.recvbuf, self.sendbuf)
        cassie_out = cassie_out_t()
        unpack_cassie_out_t(self.inbuf, cassie_out)
        return cassie_out

    def recv_wait_pd(self):
        nbytes = -1
        while nbytes != self.recvlen_pd:
            nbytes = get_newest_packet(self.sock, self.recvbuf, self.recvlen_pd,
                                         None, None)
        process_packet_header(self.packet_header_info,
                                self.recvbuf, self.sendbuf)
        state_out = state_out_t()
        unpack_state_out_t(self.inbuf, state_out)
        return state_out

    def recv_newest(self):
        nbytes = get_newest_packet(self.sock, self.recvbuf, self.recvlen,
                                     None, None)
        if nbytes != self.recvlen:
            return None
        process_packet_header(self.packet_header_info,
                                self.recvbuf, self.sendbuf)
        cassie_out = cassie_out_t()
        unpack_cassie_out_t(self.inbuf, cassie_out)
        return cassie_out

    def recv_newest_pd(self):
        nbytes = get_newest_packet(self.sock, self.recvbuf, self.recvlen_pd,
                                     None, None)
        if nbytes != self.recvlen_pd:
            return None
        process_packet_header(self.packet_header_info,
                                self.recvbuf, self.sendbuf)
        state_out = state_out_t()
        unpack_state_out_t(self.inbuf, state_out)
        return state_out

    def delay(self):
        return ord(self.packet_header_info.delay)

    def seq_num_in_diff(self):
        return ord(self.packet_header_info.seq_num_in_diff)

    def __del__(self):
        udp_close(self.sock)
