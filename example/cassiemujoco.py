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

import torch

# Get base directory
_dir_path = os.path.dirname(os.path.realpath(__file__))

# Initialize libcassiesim
# cassie_mujoco_init(str.encode(_dir_path+"/cassie.xml"))
cassie_mujoco_init(str.encode("../model/cassie.xml"))


# Interface classes
class CassieSim:
    def __init__(self, modelfile):
        self.c = cassie_sim_init(modelfile.encode('utf-8'))
        self.nv = 32
        self.nbody = 26
        self.nq = 35
        self.ngeom = 35
        
        ## Graph Conversion
        self.numNodes = 25
        self.numEdges = 22

        # all mujoco bodies, all joints, all actuated joints
        self.body_names =   ["cassie-pelvis"] + \
                            ["left-hip-roll", "left-hip-yaw", "left-hip-pitch", "left-achilles-rod", "left-knee", "left-knee-spring", "left-shin", "left-tarsus", "left-heel-spring", "left-foot-crank", "left-plantar-rod", "left-foot"] + \
                            ["right-hip-roll", "right-hip-yaw", "right-hip-pitch", "right-achilles-rod", "right-knee", "right-knee-spring", "right-shin", "right-tarsus", "right-heel-spring", "right-foot-crank", "right-plantar-rod", "right-foot"]
        self.joint_names =  ["left-hip-roll", "left-hip-yaw", "left-hip-pitch", "left-achilles-rod", "left-knee", "left-shin", "left-tarsus", "left-heel-spring", "left-foot-crank", "left-plantar-rod", "left-foot"] + \
                            ["right-hip-roll", "right-hip-yaw", "right-hip-pitch", "right-achilles-rod", "right-knee", "right-shin", "right-tarsus", "right-heel-spring", "right-foot-crank", "right-plantar-rod", "right-foot"]
        self.motor_names =  ("left-hip-roll", "left-hip-yaw", "left-hip-pitch", "left-knee", "left-foot", \
                            "right-hip-roll", "right-hip-yaw", "right-hip-pitch", "right-knee", "right-foot")
        
        # node feature tensor shape : (25, 13) : for each body, absolute position (3), quaternion (4), linear vel (3), angular vel (3)
        # edge feature tensor shape : (22, 1) : the magnitude of the action for each actuated joint, 0 for each unactuated joint
        self.EdgeConnectionTensor = self.createEdgeConnections(self.body_names, self.joint_names)

        self.dynamicNodeFeatureTensor = torch.zeros([self.numNodes, 13])
        self.dynamicEdgeFeatureTensor = torch.zeros([self.numEdges, 1])
        self.dynamicEdgeConnectionTensor = self.EdgeConnectionTensor
        self.updateDynamicGraph()

        self.staticGlobalFeatureTensor = torch.zeros([self.numNodes, 18])
        self.staticNodeFeatureTensor = torch.zeros([self.numNodes, 18])
        self.staticEdgeFeatureTensor = torch.zeros([self.numEdges, 55])
        self.createStaticGraph()

    def step(self, u):
        y = cassie_out_t()
        cassie_sim_step(self.c, y, u)
        return y

    def step_pd(self, u):
        y = state_out_t()
        cassie_sim_step_pd(self.c, y, u)
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
        return qposp[:35]

    def qvel(self):
        qvelp = cassie_sim_qvel(self.c)
        return qvelp[:32]

    def qacc(self):
        qaccp = cassie_sim_qacc(self.c)
        return qaccp[:32]

    def xpos(self, body_name):
        # print("in xquat")
        xposp = cassie_sim_xpos(self.c, body_name.encode())
        # print("got pointer")
        return xposp[:3]

    def xquat(self, body_name):
        # print("in xquat")
        xquatp = cassie_sim_xquat(self.c, body_name.encode())
        # print("got pointer")
        return xquatp[:4]

    def objectVelocity(self, body_name):
        sz = 6
        vel = np.zeros(sz)
        vel_array = (ctypes.c_double * sz)()
        # print("in xquat")
        xvelocityp = cassie_sim_objectVelocity(self.c, body_name.encode(), vel_array)
        for i in range(sz):
            vel[i] = vel_array[i]
        return vel

    def ctrl(self):
        ctrl = cassie_sim_ctrl(self.c)
        return ctrl[:10]

    # TODO: look into ways to make these more efficient
    # update tensors for dynamic graph nodes and edges
    # node feature tensor shape : (25, 13) : for each body, absolute position (3), quaternion (4), linear vel (3), angular vel (3)
    # edge feature tensor shape : (22, 1) : the magnitude of the action for each actuated joint, 0 for each unactuated joint
    def updateDynamicGraph(self):
        for i in range(self.numNodes):
            self.dynamicNodeFeatureTensor[i] = torch.cat([torch.Tensor(self.xpos(self.body_names[i]) + self.xquat(self.body_names[i])) , torch.Tensor(self.objectVelocity(self.body_names[i]))])
        ctrls = self.ctrl()
        j = 0
        for i in range(self.numEdges):
            if self.joint_names[i] in self.motor_names:
                self.dynamicEdgeFeatureTensor[i] = ctrls[j]
                j += 1
            else:
                self.dynamicEdgeFeatureTensor[i] = 0.0
        self.dynamicEdgeFeatureTensor[i].type(torch.float)

    # get the mjModel.opts.{timestep, gravity, wind, magnetic, density, viscosity, impratio, o_margin, o_solref, o_solimp,
    # collision type (and encode it as one-hot), enableflags (bit array), disableflags (bit array)}
    def mjModelopts(self):
        sz = 20
        ops = np.zeros(sz)
        opts_array = (ctypes.c_double * sz)()
        cassie_sim_get_mjModelopts(self.c, opts_array)
        for i in range(sz):
            ops[i] = opts_array[i]
        return ops

    # get the mjModel.body_{mass, pos, quat, inertia, ipos, iquat} for each body
    def mjModelbody(self):
        body = []
        sz = 18
        for body_name in self.body_names:
            body_props = np.zeros(sz)
            props_array = (ctypes.c_double * sz)()
            cassie_sim_get_mjModelbody(self.c, body_name.encode(), props_array)
            for i in range(sz):
                body_props[i] = props_array[i]
            # print("{} : {}".format(body_name, body_props[4:8]))
            body.append(body_props)
        return torch.Tensor(body)
    
    # get the mjModel.jnt_{type, axis, pos, solimp, solref, stiffness, limited, range, margin} for joint_name
    def mjModeljnt(self, joint_name):
        sz = 20
        joint_props = np.ones(sz)
        props_array = (ctypes.c_double * sz)()
        cassie_sim_get_mjModeljnt(self.c, joint_name.encode(), props_array)
        for i in range(sz):
            joint_props[i] = props_array[i]
        # print("{} : {}".format(joint_name, joint_props[4:8]))
        return torch.Tensor(joint_props)

    # get the mjModel.actuator_{{biastype (one-hot), biasprm, cranklength, ctrllimited, ctrlrange, dyntype
    #                           (one-hot), dynprm, forcelimited, forcerange, gaintype (one-hot), gainprm, gear, length0, lengthrange}
    def mjModelactuator(self, actuator_name):
        sz = 34
        actuator_props = np.zeros(sz)
        props_array = (ctypes.c_double * sz)()
        cassie_sim_get_mjModelactuator(self.c, actuator_name.encode(), props_array)
        for i in range(sz):
            actuator_props[i] = props_array[i]
        # print("{} : {}".format(joint_name, joint_props[4:8]))
        return torch.Tensor(actuator_props)

    # this only needs to be done once, it won't change at all
    # update tensors for static graph nodes and edges
    # global feature tensor shape :  (20, 1) : mjModelopt
    # node feature tensor shape   : (25, 18) : for each body, absolute position (3), quaternion (4), linear vel (3), angular vel (3)
    # edge feature tensor shape   : (22, 1+20+34) : the magnitude of the action for each actuated joint, 0 for each unactuated joint
    def createStaticGraph(self):
        self.staticGlobalFeatureTensor = self.mjModelopts()
        self.staticNodeFeatureTensor = self.mjModelbody()
        for i in range(self.numEdges):
            # if it's actuated
            if self.joint_names[i] in self.motor_names:
                self.staticEdgeFeatureTensor[i][0] = 1.0 # motorized flag
                self.staticEdgeFeatureTensor[i][1:21] = torch.zeros(20)
                self.staticEdgeFeatureTensor[i][21:] = self.mjModelactuator(self.joint_names[i])
            # it's not actuated
            else:
                self.staticEdgeFeatureTensor[i][0] = 0.0 # motorized flag
                self.staticEdgeFeatureTensor[i][1:21] = self.mjModeljnt(self.joint_names[i])
                self.staticEdgeFeatureTensor[i][21:] = torch.zeros(34)

    # TODO: figure out if edges should be directed, how to properly connect pelvis
    # return a tensor of shape [2, self.numEdges]
    def createEdgeConnections(self, node_names, edge_names):
        # initialize output tensor
        return torch.tensor([[0,1,2,3,3,5,7,8, 8, 8,10, 0,13,14,15,15,17,19,20,20,20,22],
                             [1,2,3,4,5,7,8,9,10,12,11,13,14,15,16,17,19,20,21,22,24,23]], dtype=torch.long)

    def set_time(self, time):
        timep = cassie_sim_time(self.c)
        timep[0] = time

    def set_qpos(self, qpos):
        qposp = cassie_sim_qpos(self.c)
        for i in range(min(len(qpos), 35)):
            qposp[i] = qpos[i]

    def set_qvel(self, qvel):
        qvelp = cassie_sim_qvel(self.c)
        for i in range(min(len(qvel), 32)):
            qvelp[i] = qvel[i]

    # def set_cassie_state(self, copy_state):
    #     cassie_sim_set_cassiestate(self.c, copy_state)

    def hold(self):
        cassie_sim_hold(self.c)

    def release(self):
        cassie_sim_release(self.c)

    def apply_force(self, xfrc, body_name="cassie-pelvis"):
        xfrc_array = (ctypes.c_double * 6)()
        for i in range(len(xfrc)):
            xfrc_array[i] = xfrc[i]
        cassie_sim_apply_force(self.c, xfrc_array, body_name.encode())

    def foot_force(self, force):
        frc_array = (ctypes.c_double * 12)()
        cassie_sim_foot_forces(self.c, frc_array)
        for i in range(12):
            force[i] = frc_array[i]
        #print(force)

    def foot_pos(self, pos):
        pos_array = (ctypes.c_double * 6)()
        cassie_sim_foot_positions(self.c, pos_array)
        for i in range(6):
            pos[i] = pos_array[i]

    def clear_forces(self):
        cassie_sim_clear_forces(self.c)

    def get_foot_forces(self):
        y = state_out_t()
        force = np.zeros(12)
        self.foot_force(force)
        return force[[2, 8]]

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

    def get_geom_friction(self):
        ptr = cassie_sim_geom_friction(self.c)
        ret = np.zeros(self.ngeom * 3)
        for i in range(self.ngeom * 3):
          ret[i] = ptr[i]
        return ret

    def get_geom_rgba(self):
        ptr = cassie_sim_geom_rgba(self.c)
        ret = np.zeros(self.ngeom * 4)
        for i in range(self.ngeom * 4):
          ret[i] = ptr[i]
        return ret

    def get_geom_quat(self):
        ptr = cassie_sim_geom_quat(self.c)
        ret = np.zeros(self.ngeom * 4)
        for i in range(self.ngeom * 4):
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

    def set_body_mass(self, data):
        c_arr = (ctypes.c_double * self.nbody)()

        if len(data) != self.nbody:
          print("SIZE MISMATCH SET_BODY_MASS()")
          exit(1)
        
        for i in range(self.nbody):
          c_arr[i] = data[i]

        cassie_sim_set_body_mass(self.c, c_arr)

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


    def set_geom_rgba(self, data):
        ngeom = self.ngeom * 4

        if len(data) != ngeom:
            print("SIZE MISMATCH SET_GEOM_RGBA()")
            exit(1)

        c_arr = (ctypes.c_float * ngeom)()

        for i in range(ngeom):
            c_arr[i] = data[i]

        cassie_sim_set_geom_rgba(self.c, c_arr)
    
    def set_geom_quat(self, data, name=None):
        if name is None:
            ngeom = self.ngeom * 4

            if len(data) != ngeom:
                print("SIZE MISMATCH SET_GEOM_QUAT()")
                exit(1)

            c_arr = (ctypes.c_double * ngeom)()
            #print("SETTING:")
            #print(c_arr, data)

            for i in range(ngeom):
                c_arr[i] = data[i]

            cassie_sim_set_geom_quat(self.c, c_arr)
        else:
            quat_array = (ctypes.c_double * 4)()
            for i in range(4):
                quat_array[i] = data[i]
            cassie_sim_set_geom_name_quat(self.c, name.encode(), quat_array)

    
    def set_const(self):
        cassie_sim_set_const(self.c)

    def full_reset(self):
        cassie_sim_full_reset(self.c)

    def __del__(self):
        cassie_sim_free(self.c)


class CassieGraph:
    def __init__(self, c):
        self.c = c
        self.nv = 32
        self.nbody = 26
        self.nq = 35
        self.ngeom = 35


class CassieVis:
    def __init__(self, c, modelfile):
        self.v = cassie_vis_init(c.c, modelfile.encode('utf-8'))

    def draw(self, c):
        state = cassie_vis_draw(self.v, c.c)
        # print("vis draw state:", state)
        return state

    def valid(self):
        return cassie_vis_valid(self.v)

    def ispaused(self):
        return cassie_vis_paused(self.v)

    # Applies the inputted force to the inputted body. "xfrc_apply" should contain the force/torque to 
    # apply in Cartesian coords as a 6-long array (first 3 are force, last 3 are torque). "body_name" 
    # should be a string matching a body name in the XML file. If "body_name" doesn't match an existing
    # body name, then no force will be applied. 
    def apply_force(self, xfrc_apply, body_name):
        xfrc_array = (ctypes.c_double * 6)()
        for i in range(len(xfrc_apply)):
            xfrc_array[i] = xfrc_apply[i]
        cassie_vis_apply_force(self.v, xfrc_array, body_name.encode())

    def reset(self):
        cassie_vis_full_reset(self.v)

    def __del__(self):
        cassie_vis_free(self.v)

class CassieState:
    def __init__(self):
        self.s = cassie_state_alloc()

    def time(self):
        timep = cassie_state_time(self.s)
        return timep[0]

    def qpos(self):
        qposp = cassie_state_qpos(self.s)
        return qposp[:35]

    def qvel(self):
        qvelp = cassie_state_qvel(self.s)
        return qvelp[:32]

    def set_time(self, time):
        timep = cassie_state_time(self.s)
        timep[0] = time

    def set_qpos(self, qpos):
        qposp = cassie_state_qpos(self.s)
        for i in range(min(len(qpos), 35)):
            qposp[i] = qpos[i]

    def set_qvel(self, qvel):
        qvelp = cassie_state_qvel(self.s)
        for i in range(min(len(qvel), 32)):
            qvelp[i] = qvel[i]

    def __del__(self):
        cassie_state_free(self.s)

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
