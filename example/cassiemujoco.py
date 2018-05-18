from cassiemujoco_ctypes import *
import os

# Get base directory
_dir_path = os.path.dirname(os.path.realpath(__file__))

# Initialize libcassiesim
cassie_mujoco_init(str.encode(_dir_path))

# Nicer structure names
cassie_out_t = struct_c__SA_cassie_out_t
cassie_user_in_t = struct_c__SA_cassie_user_in_t
state_out_t = struct_c__SA_state_out_t
pd_in_t = struct_c__SA_pd_in_t

# Interface classes
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

class CassieSim:
    def __init__(self):
        self.c = cassie_sim_init()

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

    def hold(self):
        cassie_sim_hold(self.c)

    def release(self):
        cassie_sim_release(self.c)

    def force(self, xfrc):
        cassie_sim_apply_force(self.c, xfrc, 0)

    def __del__(self):
        cassie_sim_free(self.c)

class CassieVis:
    def __init__(self):
        self.v = cassie_vis_init()

    def draw(self, c):
        return cassie_vis_draw(self.v, c.c)

    def valid(self):
        return cassie_vis_valid(self.v)

    def __del__(self):
        cassie_vis_free(self.v)
