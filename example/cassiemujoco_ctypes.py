# -*- coding: utf-8 -*-
#
# TARGET arch is: ['-I/usr/include/clang/6.0/include', '-Iinclude']
# WORD_SIZE is: 8
# POINTER_SIZE is: 8
# LONGDOUBLE_SIZE is: 16
#
import ctypes
import os
_dir_path = os.path.dirname(os.path.realpath(__file__))


_libraries = {}
_libraries['./libcassiemujoco.so'] = ctypes.CDLL(_dir_path + '/libcassiemujoco.so')
# if local wordsize is same as target, keep ctypes pointer function.
if ctypes.sizeof(ctypes.c_void_p) == 8:
    POINTER_T = ctypes.POINTER
else:
    # required to access _ctypes
    import _ctypes
    # Emulate a pointer class using the approriate c_int32/c_int64 type
    # The new class should have :
    # ['__module__', 'from_param', '_type_', '__dict__', '__weakref__', '__doc__']
    # but the class should be submitted to a unique instance for each base type
    # to that if A == B, POINTER_T(A) == POINTER_T(B)
    ctypes._pointer_t_type_cache = {}
    def POINTER_T(pointee):
        # a pointer should have the same length as LONG
        fake_ptr_base_type = ctypes.c_uint64 
        # specific case for c_void_p
        if pointee is None: # VOID pointer type. c_void_p.
            pointee = type(None) # ctypes.c_void_p # ctypes.c_ulong
            clsname = 'c_void'
        else:
            clsname = pointee.__name__
        if clsname in ctypes._pointer_t_type_cache:
            return ctypes._pointer_t_type_cache[clsname]
        # make template
        class _T(_ctypes._SimpleCData,):
            _type_ = 'L'
            _subtype_ = pointee
            def _sub_addr_(self):
                return self.value
            def __repr__(self):
                return '%s(%d)'%(clsname, self.value)
            def contents(self):
                raise TypeError('This is not a ctypes pointer.')
            def __init__(self, **args):
                raise TypeError('This is not a ctypes pointer. It is not instanciable.')
        _class = type('LP_%d_%s'%(8, clsname), (_T,),{}) 
        ctypes._pointer_t_type_cache[clsname] = _class
        return _class

c_int128 = ctypes.c_ubyte*16
c_uint128 = c_int128
void = None
if ctypes.sizeof(ctypes.c_longdouble) == 16:
    c_long_double_t = ctypes.c_longdouble
else:
    c_long_double_t = ctypes.c_ubyte*16



size_t = ctypes.c_uint64
socklen_t = ctypes.c_uint32
class struct_sockaddr(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('sa_family', ctypes.c_uint16),
    ('sa_data', ctypes.c_char * 14),
     ]

ssize_t = ctypes.c_int64
class struct_CassieCoreSim(ctypes.Structure):
    pass

cassie_core_sim_t = struct_CassieCoreSim
cassie_core_sim_alloc = _libraries['./libcassiemujoco.so'].cassie_core_sim_alloc
cassie_core_sim_alloc.restype = POINTER_T(struct_CassieCoreSim)
cassie_core_sim_alloc.argtypes = []
cassie_core_sim_copy = _libraries['./libcassiemujoco.so'].cassie_core_sim_copy
cassie_core_sim_copy.restype = None
cassie_core_sim_copy.argtypes = [POINTER_T(struct_CassieCoreSim), POINTER_T(struct_CassieCoreSim)]
cassie_core_sim_free = _libraries['./libcassiemujoco.so'].cassie_core_sim_free
cassie_core_sim_free.restype = None
cassie_core_sim_free.argtypes = [POINTER_T(struct_CassieCoreSim)]
cassie_core_sim_setup = _libraries['./libcassiemujoco.so'].cassie_core_sim_setup
cassie_core_sim_setup.restype = None
cassie_core_sim_setup.argtypes = [POINTER_T(struct_CassieCoreSim)]
class struct_c__SA_cassie_user_in_t(ctypes.Structure):
    pass

class struct_c__SA_cassie_out_t(ctypes.Structure):
    pass

class struct_c__SA_cassie_in_t(ctypes.Structure):
    pass

cassie_core_sim_step = _libraries['./libcassiemujoco.so'].cassie_core_sim_step
cassie_core_sim_step.restype = None
cassie_core_sim_step.argtypes = [POINTER_T(struct_CassieCoreSim), POINTER_T(struct_c__SA_cassie_user_in_t), POINTER_T(struct_c__SA_cassie_out_t), POINTER_T(struct_c__SA_cassie_in_t)]
class struct_c__SA_elmo_in_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('controlWord', ctypes.c_uint16),
    ('PADDING_0', ctypes.c_ubyte * 6),
    ('torque', ctypes.c_double),
     ]

elmo_in_t = struct_c__SA_elmo_in_t
class struct_c__SA_cassie_leg_in_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('hipRollDrive', elmo_in_t),
    ('hipYawDrive', elmo_in_t),
    ('hipPitchDrive', elmo_in_t),
    ('kneeDrive', elmo_in_t),
    ('footDrive', elmo_in_t),
     ]

cassie_leg_in_t = struct_c__SA_cassie_leg_in_t
class struct_c__SA_radio_in_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('channel', ctypes.c_int16 * 14),
     ]

radio_in_t = struct_c__SA_radio_in_t
class struct_c__SA_cassie_pelvis_in_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('radio', radio_in_t),
    ('sto', ctypes.c_bool),
    ('piezoState', ctypes.c_bool),
    ('piezoTone', ctypes.c_ubyte),
    ('PADDING_0', ctypes.c_ubyte),
     ]

cassie_pelvis_in_t = struct_c__SA_cassie_pelvis_in_t
struct_c__SA_cassie_in_t._pack_ = True # source:False
struct_c__SA_cassie_in_t._fields_ = [
    ('pelvis', cassie_pelvis_in_t),
    ('leftLeg', cassie_leg_in_t),
    ('rightLeg', cassie_leg_in_t),
]

cassie_in_t = struct_c__SA_cassie_in_t
pack_cassie_in_t = _libraries['./libcassiemujoco.so'].pack_cassie_in_t
pack_cassie_in_t.restype = None
pack_cassie_in_t.argtypes = [POINTER_T(struct_c__SA_cassie_in_t), POINTER_T(ctypes.c_ubyte)]
unpack_cassie_in_t = _libraries['./libcassiemujoco.so'].unpack_cassie_in_t
unpack_cassie_in_t.restype = None
unpack_cassie_in_t.argtypes = [POINTER_T(ctypes.c_ubyte), POINTER_T(struct_c__SA_cassie_in_t)]
DiagnosticCodes = ctypes.c_int16
class struct_c__SA_battery_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('dataGood', ctypes.c_bool),
    ('PADDING_0', ctypes.c_ubyte * 7),
    ('stateOfCharge', ctypes.c_double),
    ('voltage', ctypes.c_double * 12),
    ('current', ctypes.c_double),
    ('temperature', ctypes.c_double * 4),
     ]

battery_out_t = struct_c__SA_battery_out_t
class struct_c__SA_cassie_joint_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('position', ctypes.c_double),
    ('velocity', ctypes.c_double),
     ]

cassie_joint_out_t = struct_c__SA_cassie_joint_out_t
class struct_c__SA_elmo_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('statusWord', ctypes.c_uint16),
    ('PADDING_0', ctypes.c_ubyte * 6),
    ('position', ctypes.c_double),
    ('velocity', ctypes.c_double),
    ('torque', ctypes.c_double),
    ('driveTemperature', ctypes.c_double),
    ('dcLinkVoltage', ctypes.c_double),
    ('torqueLimit', ctypes.c_double),
    ('gearRatio', ctypes.c_double),
     ]

elmo_out_t = struct_c__SA_elmo_out_t
class struct_c__SA_cassie_leg_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('hipRollDrive', elmo_out_t),
    ('hipYawDrive', elmo_out_t),
    ('hipPitchDrive', elmo_out_t),
    ('kneeDrive', elmo_out_t),
    ('footDrive', elmo_out_t),
    ('shinJoint', cassie_joint_out_t),
    ('tarsusJoint', cassie_joint_out_t),
    ('footJoint', cassie_joint_out_t),
    ('medullaCounter', ctypes.c_ubyte),
    ('PADDING_0', ctypes.c_ubyte),
    ('medullaCpuLoad', ctypes.c_uint16),
    ('reedSwitchState', ctypes.c_bool),
    ('PADDING_1', ctypes.c_ubyte * 3),
     ]

cassie_leg_out_t = struct_c__SA_cassie_leg_out_t
class struct_c__SA_radio_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('radioReceiverSignalGood', ctypes.c_bool),
    ('receiverMedullaSignalGood', ctypes.c_bool),
    ('PADDING_0', ctypes.c_ubyte * 6),
    ('channel', ctypes.c_double * 16),
     ]

radio_out_t = struct_c__SA_radio_out_t
class struct_c__SA_target_pc_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('etherCatStatus', ctypes.c_int32 * 6),
    ('etherCatNotifications', ctypes.c_int32 * 21),
    ('PADDING_0', ctypes.c_ubyte * 4),
    ('taskExecutionTime', ctypes.c_double),
    ('overloadCounter', ctypes.c_uint32),
    ('PADDING_1', ctypes.c_ubyte * 4),
    ('cpuTemperature', ctypes.c_double),
     ]

target_pc_out_t = struct_c__SA_target_pc_out_t
class struct_c__SA_vectornav_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('dataGood', ctypes.c_bool),
    ('PADDING_0', ctypes.c_ubyte),
    ('vpeStatus', ctypes.c_uint16),
    ('PADDING_1', ctypes.c_ubyte * 4),
    ('pressure', ctypes.c_double),
    ('temperature', ctypes.c_double),
    ('magneticField', ctypes.c_double * 3),
    ('angularVelocity', ctypes.c_double * 3),
    ('linearAcceleration', ctypes.c_double * 3),
    ('orientation', ctypes.c_double * 4),
     ]

vectornav_out_t = struct_c__SA_vectornav_out_t
class struct_c__SA_cassie_pelvis_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('targetPc', target_pc_out_t),
    ('battery', battery_out_t),
    ('radio', radio_out_t),
    ('vectorNav', vectornav_out_t),
    ('medullaCounter', ctypes.c_ubyte),
    ('PADDING_0', ctypes.c_ubyte),
    ('medullaCpuLoad', ctypes.c_uint16),
    ('bleederState', ctypes.c_bool),
    ('leftReedSwitchState', ctypes.c_bool),
    ('rightReedSwitchState', ctypes.c_bool),
    ('PADDING_1', ctypes.c_ubyte),
    ('vtmTemperature', ctypes.c_double),
     ]

cassie_pelvis_out_t = struct_c__SA_cassie_pelvis_out_t
struct_c__SA_cassie_out_t._pack_ = True # source:False
struct_c__SA_cassie_out_t._fields_ = [
    ('pelvis', cassie_pelvis_out_t),
    ('leftLeg', cassie_leg_out_t),
    ('rightLeg', cassie_leg_out_t),
    ('isCalibrated', ctypes.c_bool),
    ('PADDING_0', ctypes.c_ubyte),
    ('messages', ctypes.c_int16 * 4),
    ('PADDING_1', ctypes.c_ubyte * 6),
]

cassie_out_t = struct_c__SA_cassie_out_t
pack_cassie_out_t = _libraries['./libcassiemujoco.so'].pack_cassie_out_t
pack_cassie_out_t.restype = None
pack_cassie_out_t.argtypes = [POINTER_T(struct_c__SA_cassie_out_t), POINTER_T(ctypes.c_ubyte)]
unpack_cassie_out_t = _libraries['./libcassiemujoco.so'].unpack_cassie_out_t
unpack_cassie_out_t.restype = None
unpack_cassie_out_t.argtypes = [POINTER_T(ctypes.c_ubyte), POINTER_T(struct_c__SA_cassie_out_t)]
struct_c__SA_cassie_user_in_t._pack_ = True # source:False
struct_c__SA_cassie_user_in_t._fields_ = [
    ('torque', ctypes.c_double * 10),
    ('telemetry', ctypes.c_int16 * 9),
    ('PADDING_0', ctypes.c_ubyte * 6),
]

cassie_user_in_t = struct_c__SA_cassie_user_in_t
pack_cassie_user_in_t = _libraries['./libcassiemujoco.so'].pack_cassie_user_in_t
pack_cassie_user_in_t.restype = None
pack_cassie_user_in_t.argtypes = [POINTER_T(struct_c__SA_cassie_user_in_t), POINTER_T(ctypes.c_ubyte)]
unpack_cassie_user_in_t = _libraries['./libcassiemujoco.so'].unpack_cassie_user_in_t
unpack_cassie_user_in_t.restype = None
unpack_cassie_user_in_t.argtypes = [POINTER_T(ctypes.c_ubyte), POINTER_T(struct_c__SA_cassie_user_in_t)]
class struct_cassie_sim(ctypes.Structure):
    pass

cassie_sim_t = struct_cassie_sim
class struct_cassie_vis(ctypes.Structure):
    pass

cassie_vis_t = struct_cassie_vis
class struct_cassie_state(ctypes.Structure):
    pass

cassie_state_t = struct_cassie_state
cassie_mujoco_init = _libraries['./libcassiemujoco.so'].cassie_mujoco_init
cassie_mujoco_init.restype = ctypes.c_bool
cassie_mujoco_init.argtypes = [POINTER_T(ctypes.c_char)]
cassie_cleanup = _libraries['./libcassiemujoco.so'].cassie_cleanup
cassie_cleanup.restype = None
cassie_cleanup.argtypes = []
cassie_reload_xml = _libraries['./libcassiemujoco.so'].cassie_reload_xml
cassie_reload_xml.restype = ctypes.c_bool
cassie_reload_xml.argtypes = [ctypes.c_char_p]
cassie_sim_init = _libraries['./libcassiemujoco.so'].cassie_sim_init
cassie_sim_init.restype = POINTER_T(struct_cassie_sim)
cassie_sim_init.argtypes = [ctypes.c_char_p, ctypes.c_bool]
cassie_sim_duplicate = _libraries['./libcassiemujoco.so'].cassie_sim_duplicate
cassie_sim_duplicate.restype = POINTER_T(struct_cassie_sim)
cassie_sim_duplicate.argtypes = [POINTER_T(struct_cassie_sim)]
cassie_sim_copy = _libraries['./libcassiemujoco.so'].cassie_sim_copy
cassie_sim_copy.restype = None
cassie_sim_copy.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(struct_cassie_sim)]
cassie_sim_free = _libraries['./libcassiemujoco.so'].cassie_sim_free
cassie_sim_free.restype = None
cassie_sim_free.argtypes = [POINTER_T(struct_cassie_sim)]
cassie_sim_step_ethercat = _libraries['./libcassiemujoco.so'].cassie_sim_step_ethercat
cassie_sim_step_ethercat.restype = None
cassie_sim_step_ethercat.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(struct_c__SA_cassie_out_t), POINTER_T(struct_c__SA_cassie_in_t)]
cassie_sim_step = _libraries['./libcassiemujoco.so'].cassie_sim_step
cassie_sim_step.restype = None
cassie_sim_step.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(struct_c__SA_cassie_out_t), POINTER_T(struct_c__SA_cassie_user_in_t)]
class struct_c__SA_state_out_t(ctypes.Structure):
    pass

class struct_c__SA_pd_in_t(ctypes.Structure):
    pass

class struct_c__SA_joint_filter_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('x', ctypes.c_double * 4),
    ('y', ctypes.c_double * 3),
    ]
joint_filter_t = struct_c__SA_joint_filter_t

class struct_c__SA_drive_filter_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('x', ctypes.c_double * 9),
    ]
drive_filter_t = struct_c__SA_drive_filter_t

cassie_sim_step_pd = _libraries['./libcassiemujoco.so'].cassie_sim_step_pd
cassie_sim_step_pd.restype = None
cassie_sim_step_pd.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(struct_c__SA_state_out_t), POINTER_T(struct_c__SA_pd_in_t)]

cassie_integrate_pos = _libraries['./libcassiemujoco.so'].cassie_integrate_pos
cassie_integrate_pos.restype = None
cassie_integrate_pos.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(struct_c__SA_state_out_t)]

cassie_sim_time = _libraries['./libcassiemujoco.so'].cassie_sim_time
cassie_sim_time.restype = POINTER_T(ctypes.c_double)
cassie_sim_time.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_qpos = _libraries['./libcassiemujoco.so'].cassie_sim_qpos
cassie_sim_qpos.restype = POINTER_T(ctypes.c_double)
cassie_sim_qpos.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_qvel = _libraries['./libcassiemujoco.so'].cassie_sim_qvel
cassie_sim_qvel.restype = POINTER_T(ctypes.c_double)
cassie_sim_qvel.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_qacc = _libraries['./libcassiemujoco.so'].cassie_sim_qacc
cassie_sim_qacc.restype = POINTER_T(ctypes.c_double)
cassie_sim_qacc.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_mjmodel = _libraries['./libcassiemujoco.so'].cassie_sim_mjmodel
cassie_sim_mjmodel.restype = POINTER_T(None)
cassie_sim_mjmodel.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_mjdata = _libraries['./libcassiemujoco.so'].cassie_sim_mjdata
cassie_sim_mjdata.restype = POINTER_T(None)
cassie_sim_mjdata.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_check_obstacle_collision = _libraries['./libcassiemujoco.so'].cassie_sim_check_obstacle_collision
cassie_sim_check_obstacle_collision.restype = ctypes.c_bool
cassie_sim_check_obstacle_collision.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_check_self_collision = _libraries['./libcassiemujoco.so'].cassie_sim_check_self_collision
cassie_sim_check_self_collision.restype = ctypes.c_bool
cassie_sim_check_self_collision.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_foot_forces = _libraries['./libcassiemujoco.so'].cassie_sim_foot_forces
cassie_sim_foot_forces.restype = None
cassie_sim_foot_forces.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_double * 12]

cassie_sim_heeltoe_forces = _libraries['./libcassiemujoco.so'].cassie_sim_heeltoe_forces
cassie_sim_heeltoe_forces.restype = None
cassie_sim_heeltoe_forces.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_double * 6, ctypes.c_double * 6]

cassie_sim_foot_positions = _libraries['./libcassiemujoco.so'].cassie_sim_foot_positions
cassie_sim_foot_positions.restype = None
cassie_sim_foot_positions.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_double * 6]

cassie_sim_foot_velocities = _libraries['./libcassiemujoco.so'].cassie_sim_foot_velocities
cassie_sim_foot_velocities.restype = None
cassie_sim_foot_velocities.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_double * 12]

cassie_sim_cm_position = _libraries['./libcassiemujoco.so'].cassie_sim_cm_position
cassie_sim_cm_position.restype = None
cassie_sim_cm_position.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_double * 3]

cassie_sim_cm_velocity = _libraries['./libcassiemujoco.so'].cassie_sim_cm_velocity
cassie_sim_cm_velocity.restype = None
cassie_sim_cm_velocity.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_double * 3]

cassie_sim_centroid_inertia = _libraries['./libcassiemujoco.so'].cassie_sim_centroid_inertia
cassie_sim_centroid_inertia.restype = None
cassie_sim_centroid_inertia.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_double * 9]

cassie_sim_angular_momentum = _libraries['./libcassiemujoco.so'].cassie_sim_angular_momentum
cassie_sim_angular_momentum.restype = None
cassie_sim_angular_momentum.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_double * 3]

cassie_sim_full_mass_matrix = _libraries['./libcassiemujoco.so'].cassie_sim_full_mass_matrix
cassie_sim_full_mass_matrix.restype = None
cassie_sim_full_mass_matrix.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_double * (32*32)]

cassie_sim_minimal_mass_matrix = _libraries['./libcassiemujoco.so'].cassie_sim_minimal_mass_matrix
cassie_sim_minimal_mass_matrix.restype = None
cassie_sim_minimal_mass_matrix.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_double * (16*16)]

cassie_sim_loop_constraint_info = _libraries['./libcassiemujoco.so'].cassie_sim_loop_constraint_info
cassie_sim_loop_constraint_info.restype = None
cassie_sim_loop_constraint_info.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_double * (6*32), ctypes.c_double * (6)]

cassie_sim_foot_quat = _libraries['./libcassiemujoco.so'].cassie_sim_foot_orient
cassie_sim_foot_quat.restype = None
cassie_sim_foot_quat.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_double * 4]

cassie_sim_body_vel = _libraries['./libcassiemujoco.so'].cassie_sim_body_velocities
cassie_sim_body_vel.restype = None
cassie_sim_body_vel.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_double * 6, ctypes.c_char_p]

cassie_sim_apply_force = _libraries['./libcassiemujoco.so'].cassie_sim_apply_force
cassie_sim_apply_force.restype = None
cassie_sim_apply_force.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_double * 6, ctypes.c_char_p]

cassie_sim_xpos = _libraries['./libcassiemujoco.so'].cassie_sim_xpos
cassie_sim_xpos.restype = POINTER_T(ctypes.c_double)
cassie_sim_xpos.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_char_p]

cassie_sim_xquat = _libraries['./libcassiemujoco.so'].cassie_sim_xquat
cassie_sim_xquat.restype = POINTER_T(ctypes.c_double)
cassie_sim_xquat.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_char_p]

cassie_sim_clear_forces = _libraries['./libcassiemujoco.so'].cassie_sim_clear_forces
cassie_sim_clear_forces.restype = None
cassie_sim_clear_forces.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_hold = _libraries['./libcassiemujoco.so'].cassie_sim_hold
cassie_sim_hold.restype = None
cassie_sim_hold.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_release = _libraries['./libcassiemujoco.so'].cassie_sim_release
cassie_sim_release.restype = None
cassie_sim_release.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_radio = _libraries['./libcassiemujoco.so'].cassie_sim_radio
cassie_sim_radio.restype = None
cassie_sim_radio.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_double * 16]

cassie_sim_full_reset = _libraries['./libcassiemujoco.so'].cassie_sim_full_reset
cassie_sim_full_reset.restype = None
cassie_sim_full_reset.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_get_hfield_nrow = _libraries['./libcassiemujoco.so'].cassie_sim_get_hfield_nrow
cassie_sim_get_hfield_nrow.restype = ctypes.c_int32
cassie_sim_get_hfield_nrow.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_get_hfield_ncol = _libraries['./libcassiemujoco.so'].cassie_sim_get_hfield_ncol
cassie_sim_get_hfield_ncol.restype = ctypes.c_int32
cassie_sim_get_hfield_ncol.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_get_nhfielddata = _libraries['./libcassiemujoco.so'].cassie_sim_get_nhfielddata
cassie_sim_get_nhfielddata.restype = ctypes.c_int32
cassie_sim_get_nhfielddata.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_get_hfield_size = _libraries['./libcassiemujoco.so'].cassie_sim_get_hfield_size
cassie_sim_get_hfield_size.restype = POINTER_T(ctypes.c_double)
cassie_sim_get_hfield_size.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_set_hfield_size = _libraries['./libcassiemujoco.so'].cassie_sim_set_hfield_size
cassie_sim_set_hfield_size.restype = None
cassie_sim_set_hfield_size.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_double * 4]

cassie_sim_hfielddata = _libraries['./libcassiemujoco.so'].cassie_sim_hfielddata
cassie_sim_hfielddata.restype = POINTER_T(ctypes.c_float)
cassie_sim_hfielddata.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_set_hfielddata = _libraries['./libcassiemujoco.so'].cassie_sim_set_hfielddata
cassie_sim_set_hfielddata.restype = None
cassie_sim_set_hfielddata.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(ctypes.c_float)]

cassie_vis_init = _libraries['./libcassiemujoco.so'].cassie_vis_init
cassie_vis_init.restype = POINTER_T(struct_cassie_vis)
cassie_vis_init.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_char_p,ctypes.c_bool]

cassie_vis_extent = _libraries['./libcassiemujoco.so'].cassie_vis_extent
cassie_vis_extent.restype = ctypes.c_float
cassie_vis_extent.argtypes = [POINTER_T(struct_cassie_vis)]

cassie_vis_znear = _libraries['./libcassiemujoco.so'].cassie_vis_znear
cassie_vis_znear.restype = ctypes.c_float
cassie_vis_znear.argtypes = [POINTER_T(struct_cassie_vis)]

cassie_vis_zfar = _libraries['./libcassiemujoco.so'].cassie_vis_zfar
cassie_vis_zfar.restype = ctypes.c_float
cassie_vis_zfar.argtypes = [POINTER_T(struct_cassie_vis)]

cassie_vis_close = _libraries['./libcassiemujoco.so'].cassie_vis_close
cassie_vis_close.restype = None
cassie_vis_close.argtypes = [POINTER_T(struct_cassie_vis)]

cassie_vis_free = _libraries['./libcassiemujoco.so'].cassie_vis_free
cassie_vis_free.restype = None
cassie_vis_free.argtypes = [POINTER_T(struct_cassie_vis)]

cassie_vis_draw = _libraries['./libcassiemujoco.so'].cassie_vis_draw
cassie_vis_draw.restype = ctypes.c_bool
cassie_vis_draw.argtypes = [POINTER_T(struct_cassie_vis), POINTER_T(struct_cassie_sim)]

cassie_vis_set_cam = _libraries['./libcassiemujoco.so'].cassie_vis_set_cam
cassie_vis_set_cam.restype = None
cassie_vis_set_cam.argtypes = [POINTER_T(struct_cassie_vis), ctypes.c_char_p, ctypes.c_double, ctypes.c_double, ctypes.c_double]

cassie_vis_window_resize = _libraries['./libcassiemujoco.so'].cassie_vis_window_resize
cassie_vis_window_resize.restype = None
cassie_vis_window_resize.argtypes = [POINTER_T(struct_cassie_vis), ctypes.c_int32, ctypes.c_int32]

cassie_vis_attach_cam = _libraries['./libcassiemujoco.so'].cassie_vis_attach_cam
cassie_vis_attach_cam.restype = None
cassie_vis_attach_cam.argtypes = [POINTER_T(struct_cassie_vis), ctypes.c_char_p]

cassie_vis_draw_depth = _libraries['./libcassiemujoco.so'].cassie_vis_draw_depth
cassie_vis_draw_depth.restype = POINTER_T(ctypes.c_float)
cassie_vis_draw_depth.argtypes = [POINTER_T(struct_cassie_vis), POINTER_T(struct_cassie_sim), ctypes.c_int32, ctypes.c_int32]

cassie_vis_get_depth_size = _libraries['./libcassiemujoco.so'].cassie_vis_get_depth_size
cassie_vis_get_depth_size.restype = ctypes.c_int32
cassie_vis_get_depth_size.argtypes = [POINTER_T(struct_cassie_vis), ctypes.c_int32, ctypes.c_int32]

cassie_vis_init_depth = _libraries['./libcassiemujoco.so'].cassie_vis_init_depth
cassie_vis_init_depth.restype = None
cassie_vis_init_depth.argtypes = [POINTER_T(struct_cassie_vis), ctypes.c_int, ctypes.c_int]

cassie_vis_valid = _libraries['./libcassiemujoco.so'].cassie_vis_valid
cassie_vis_valid.restype = ctypes.c_bool
cassie_vis_valid.argtypes = [POINTER_T(struct_cassie_vis)]

cassie_vis_paused = _libraries['./libcassiemujoco.so'].cassie_vis_paused
cassie_vis_paused.restype = ctypes.c_bool
cassie_vis_paused.argtypes = [POINTER_T(struct_cassie_vis)]

cassie_vis_apply_force = _libraries['./libcassiemujoco.so'].cassie_vis_apply_force
cassie_vis_apply_force.restype = None
cassie_vis_apply_force.argtypes = [POINTER_T(struct_cassie_vis), POINTER_T(ctypes.c_double), ctypes.c_char_p]

cassie_vis_add_marker = _libraries['./libcassiemujoco.so'].cassie_vis_add_marker
cassie_vis_add_marker.restype = None
cassie_vis_add_marker.argtypes = [POINTER_T(struct_cassie_vis), ctypes.c_double * 3, ctypes.c_double * 3, ctypes.c_double * 4, ctypes.c_double * 9]

cassie_vis_remove_marker = _libraries['./libcassiemujoco.so'].cassie_vis_remove_marker
cassie_vis_remove_marker.restype = None
cassie_vis_remove_marker.argtypes = [POINTER_T(struct_cassie_vis), ctypes.c_int]

cassie_vis_clear_markers = _libraries['./libcassiemujoco.so'].cassie_vis_clear_markers
cassie_vis_clear_markers.restype = None
cassie_vis_clear_markers.argtypes = [POINTER_T(struct_cassie_vis)]

cassie_vis_update_marker_pos = _libraries['./libcassiemujoco.so'].cassie_vis_update_marker_pos
cassie_vis_update_marker_pos.restype = None
cassie_vis_update_marker_pos.argtypes = [POINTER_T(struct_cassie_vis), ctypes.c_int, ctypes.c_double * 3]

cassie_vis_update_marker_size = _libraries['./libcassiemujoco.so'].cassie_vis_update_marker_size
cassie_vis_update_marker_size.restype = None
cassie_vis_update_marker_size.argtypes = [POINTER_T(struct_cassie_vis), ctypes.c_int, ctypes.c_double * 3]

cassie_vis_update_marker_rgba = _libraries['./libcassiemujoco.so'].cassie_vis_update_marker_rgba
cassie_vis_update_marker_rgba.restype = None
cassie_vis_update_marker_rgba.argtypes = [POINTER_T(struct_cassie_vis), ctypes.c_int, ctypes.c_double * 4]

cassie_vis_update_marker_orient = _libraries['./libcassiemujoco.so'].cassie_vis_update_marker_orient
cassie_vis_update_marker_orient.restype = None
cassie_vis_update_marker_orient.argtypes = [POINTER_T(struct_cassie_vis), ctypes.c_int, ctypes.c_double * 9]

cassie_vis_full_reset = _libraries['./libcassiemujoco.so'].cassie_vis_full_reset
cassie_vis_full_reset.restype = None
cassie_vis_full_reset.argtypes = [POINTER_T(struct_cassie_vis)]

cassie_vis_remakeSceneCon = _libraries['./libcassiemujoco.so'].cassie_vis_remakeSceneCon
cassie_vis_remakeSceneCon.restype = None
cassie_vis_remakeSceneCon.argtypes = [POINTER_T(struct_cassie_vis)]

cassie_vis_init_recording = _libraries['./libcassiemujoco.so'].cassie_vis_init_recording
cassie_vis_init_recording.restype = None
cassie_vis_init_recording.argtypes = [POINTER_T(struct_cassie_vis), ctypes.c_char_p, ctypes.c_int32, ctypes.c_int32]

cassie_vis_record_frame = _libraries['./libcassiemujoco.so'].cassie_vis_record_frame
cassie_vis_record_frame.restype = None
cassie_vis_record_frame.argtypes = [POINTER_T(struct_cassie_vis)]

cassie_vis_close_recording = _libraries['./libcassiemujoco.so'].cassie_vis_close_recording
cassie_vis_close_recording.restype = None
cassie_vis_close_recording.argtypes = [POINTER_T(struct_cassie_vis)]

cassie_state_alloc = _libraries['./libcassiemujoco.so'].cassie_state_alloc
cassie_state_alloc.restype = POINTER_T(struct_cassie_state)
cassie_state_alloc.argtypes = []

cassie_state_duplicate = _libraries['./libcassiemujoco.so'].cassie_state_duplicate
cassie_state_duplicate.restype = POINTER_T(struct_cassie_state)
cassie_state_duplicate.argtypes = [POINTER_T(struct_cassie_state)]

cassie_state_copy = _libraries['./libcassiemujoco.so'].cassie_state_copy
cassie_state_copy.restype = None
cassie_state_copy.argtypes = [POINTER_T(struct_cassie_state), POINTER_T(struct_cassie_state)]

cassie_state_free = _libraries['./libcassiemujoco.so'].cassie_state_free
cassie_state_free.restype = None
cassie_state_free.argtypes = [POINTER_T(struct_cassie_state)]

cassie_state_time = _libraries['./libcassiemujoco.so'].cassie_state_time
cassie_state_time.restype = POINTER_T(ctypes.c_double)
cassie_state_time.argtypes = [POINTER_T(struct_cassie_state)]

cassie_state_qpos = _libraries['./libcassiemujoco.so'].cassie_state_qpos
cassie_state_qpos.restype = POINTER_T(ctypes.c_double)
cassie_state_qpos.argtypes = [POINTER_T(struct_cassie_state)]

cassie_state_qvel = _libraries['./libcassiemujoco.so'].cassie_state_qvel
cassie_state_qvel.restype = POINTER_T(ctypes.c_double)
cassie_state_qvel.argtypes = [POINTER_T(struct_cassie_state)]

cassie_get_state = _libraries['./libcassiemujoco.so'].cassie_get_state
cassie_get_state.restype = None
cassie_get_state.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(struct_cassie_state)]

cassie_set_state = _libraries['./libcassiemujoco.so'].cassie_set_state
cassie_set_state.restype = None
cassie_set_state.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(struct_cassie_state)]

cassie_sim_read_rangefinder = _libraries['./libcassiemujoco.so'].cassie_sim_read_rangefinder
cassie_sim_read_rangefinder.restype = None
cassie_sim_read_rangefinder.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_double * 6]

#cassie_sim_foot_positions.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_double * 6]

cassie_sim_dof_damping = _libraries['./libcassiemujoco.so'].cassie_sim_dof_damping
cassie_sim_dof_damping.restype = POINTER_T(ctypes.c_double)
cassie_sim_dof_damping.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_set_dof_damping = _libraries['./libcassiemujoco.so'].cassie_sim_set_dof_damping
cassie_sim_set_dof_damping.restype = None
cassie_sim_set_dof_damping.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(ctypes.c_double)]

cassie_sim_body_mass = _libraries['./libcassiemujoco.so'].cassie_sim_body_mass
cassie_sim_body_mass.restype = POINTER_T(ctypes.c_double)
cassie_sim_body_mass.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_set_body_mass = _libraries['./libcassiemujoco.so'].cassie_sim_set_body_mass
cassie_sim_set_body_mass.restype = None
cassie_sim_set_body_mass.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(ctypes.c_double)]

cassie_sim_set_body_name_mass = _libraries['./libcassiemujoco.so'].cassie_sim_set_body_name_mass
cassie_sim_set_body_name_mass.restype = None
cassie_sim_set_body_name_mass.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_char_p, ctypes.c_double]

cassie_sim_body_ipos = _libraries['./libcassiemujoco.so'].cassie_sim_body_ipos
cassie_sim_body_ipos.restype = POINTER_T(ctypes.c_double)
cassie_sim_body_ipos.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_get_body_name_pos = _libraries['./libcassiemujoco.so'].cassie_sim_get_body_name_pos
cassie_sim_get_body_name_pos.restype = POINTER_T(ctypes.c_double)
cassie_sim_get_body_name_pos.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_char_p]

cassie_sim_set_body_name_pos = _libraries['./libcassiemujoco.so'].cassie_sim_set_body_name_pos
cassie_sim_set_body_name_pos.restype = None
cassie_sim_set_body_name_pos.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_char_p, POINTER_T(ctypes.c_double)]

cassie_sim_set_body_ipos = _libraries['./libcassiemujoco.so'].cassie_sim_set_body_ipos
cassie_sim_set_body_ipos.restype = None
cassie_sim_set_body_ipos.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(ctypes.c_double)]

cassie_sim_geom_friction = _libraries['./libcassiemujoco.so'].cassie_sim_geom_friction
cassie_sim_geom_friction.restype = POINTER_T(ctypes.c_double)
cassie_sim_geom_friction.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_set_geom_friction = _libraries['./libcassiemujoco.so'].cassie_sim_set_geom_friction
cassie_sim_set_geom_friction.restype = None
cassie_sim_set_geom_friction.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(ctypes.c_double)]

cassie_sim_set_geom_name_friction = _libraries['./libcassiemujoco.so'].cassie_sim_set_geom_name_friction
cassie_sim_set_geom_name_friction.restype = None
cassie_sim_set_geom_name_friction.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_char_p, POINTER_T(ctypes.c_double)]

cassie_sim_geom_rgba = _libraries['./libcassiemujoco.so'].cassie_sim_geom_rgba
cassie_sim_geom_rgba.restype = POINTER_T(ctypes.c_float)
cassie_sim_geom_rgba.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_geom_name_rgba = _libraries['./libcassiemujoco.so'].cassie_sim_geom_name_rgba
cassie_sim_geom_name_rgba.restype = POINTER_T(ctypes.c_float)
cassie_sim_geom_name_rgba.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_char_p]

cassie_sim_set_geom_rgba = _libraries['./libcassiemujoco.so'].cassie_sim_set_geom_rgba
cassie_sim_set_geom_rgba.restype = None
cassie_sim_set_geom_rgba.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(ctypes.c_float)]

cassie_sim_set_geom_name_rgba = _libraries['./libcassiemujoco.so'].cassie_sim_set_geom_name_rgba
cassie_sim_set_geom_name_rgba.restype = None
cassie_sim_set_geom_name_rgba.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_char_p, POINTER_T(ctypes.c_float)]

cassie_sim_geom_quat = _libraries['./libcassiemujoco.so'].cassie_sim_geom_quat
cassie_sim_geom_quat.restype = POINTER_T(ctypes.c_double)
cassie_sim_geom_quat.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_geom_name_quat = _libraries['./libcassiemujoco.so'].cassie_sim_geom_name_quat
cassie_sim_geom_name_quat.restype = POINTER_T(ctypes.c_double)
cassie_sim_geom_name_quat.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_char_p]

cassie_sim_set_geom_quat = _libraries['./libcassiemujoco.so'].cassie_sim_set_geom_quat
cassie_sim_set_geom_quat.restype = None
cassie_sim_set_geom_quat.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(ctypes.c_double)]

cassie_sim_set_geom_name_quat = _libraries['./libcassiemujoco.so'].cassie_sim_set_geom_name_quat
cassie_sim_set_geom_name_quat.restype = None
cassie_sim_set_geom_name_quat.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_char_p, POINTER_T(ctypes.c_double)]

cassie_sim_geom_pos = _libraries['./libcassiemujoco.so'].cassie_sim_geom_pos
cassie_sim_geom_pos.restype = POINTER_T(ctypes.c_double)
cassie_sim_geom_pos.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_geom_name_pos = _libraries['./libcassiemujoco.so'].cassie_sim_geom_name_pos
cassie_sim_geom_name_pos.restype = POINTER_T(ctypes.c_double)
cassie_sim_geom_name_pos.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_char_p]

cassie_sim_set_geom_pos = _libraries['./libcassiemujoco.so'].cassie_sim_set_geom_pos
cassie_sim_set_geom_pos.restype = None
cassie_sim_set_geom_pos.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(ctypes.c_double)]

cassie_sim_set_geom_name_pos = _libraries['./libcassiemujoco.so'].cassie_sim_set_geom_name_pos
cassie_sim_set_geom_name_pos.restype = None
cassie_sim_set_geom_name_pos.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_char_p, POINTER_T(ctypes.c_double)]

cassie_sim_geom_size = _libraries['./libcassiemujoco.so'].cassie_sim_geom_size
cassie_sim_geom_size.restype = POINTER_T(ctypes.c_double)
cassie_sim_geom_size.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_geom_name_size = _libraries['./libcassiemujoco.so'].cassie_sim_geom_name_size
cassie_sim_geom_name_size.restype = POINTER_T(ctypes.c_double)
cassie_sim_geom_name_size.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_char_p]

cassie_sim_set_geom_size = _libraries['./libcassiemujoco.so'].cassie_sim_set_geom_size
cassie_sim_set_geom_size.restype = None
cassie_sim_set_geom_size.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(ctypes.c_double)]

cassie_sim_set_geom_name_size = _libraries['./libcassiemujoco.so'].cassie_sim_set_geom_name_size
cassie_sim_set_geom_name_size.restype = None
cassie_sim_set_geom_name_size.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_char_p, POINTER_T(ctypes.c_double)]

cassie_sim_set_const = _libraries['./libcassiemujoco.so'].cassie_sim_set_const
cassie_sim_set_const.restype = None
cassie_sim_set_const.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_params = _libraries['./libcassiemujoco.so'].cassie_sim_params
cassie_sim_params.restype = None
cassie_sim_params.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(ctypes.c_int32)]

cassie_sim_joint_filter = _libraries['./libcassiemujoco.so'].cassie_sim_joint_filter
cassie_sim_joint_filter.restype = POINTER_T(joint_filter_t)
cassie_sim_joint_filter.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_get_joint_filter = _libraries['./libcassiemujoco.so'].cassie_sim_get_joint_filter
cassie_sim_get_joint_filter.restype = None
cassie_sim_get_joint_filter.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(ctypes.c_double)]

cassie_sim_set_joint_filter = _libraries['./libcassiemujoco.so'].cassie_sim_set_joint_filter
cassie_sim_set_joint_filter.restype = None
cassie_sim_set_joint_filter.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(ctypes.c_double), POINTER_T(ctypes.c_double)]

cassie_sim_drive_filter = _libraries['./libcassiemujoco.so'].cassie_sim_drive_filter
cassie_sim_drive_filter.restype = POINTER_T(drive_filter_t)
cassie_sim_drive_filter.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_get_drive_filter = _libraries['./libcassiemujoco.so'].cassie_sim_drive_filter
cassie_sim_get_drive_filter.restype = None
cassie_sim_get_drive_filter.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(ctypes.c_double)]

cassie_sim_set_drive_filter = _libraries['./libcassiemujoco.so'].cassie_sim_set_drive_filter
cassie_sim_set_drive_filter.restype = None
cassie_sim_set_drive_filter.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(ctypes.c_double)]

cassie_sim_torque_delay = _libraries['./libcassiemujoco.so'].cassie_sim_torque_delay
cassie_sim_torque_delay.restype = None
cassie_sim_torque_delay.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(ctypes.c_double)]

cassie_sim_set_torque_delay = _libraries['./libcassiemujoco.so'].cassie_sim_set_torque_delay
cassie_sim_set_torque_delay.restype = None
cassie_sim_set_torque_delay.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(ctypes.c_double)]
cassie_sim_nv = _libraries['./libcassiemujoco.so'].cassie_sim_nv
cassie_sim_nv.restype = ctypes.c_int32
cassie_sim_nv.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_nbody = _libraries['./libcassiemujoco.so'].cassie_sim_nbody
cassie_sim_nbody.restype = ctypes.c_int32
cassie_sim_nbody.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_ngeom = _libraries['./libcassiemujoco.so'].cassie_sim_ngeom
cassie_sim_ngeom.restype = ctypes.c_int32
cassie_sim_ngeom.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_nq = _libraries['./libcassiemujoco.so'].cassie_sim_nq
cassie_sim_nq.restype = ctypes.c_int32
cassie_sim_nq.argtypes = [POINTER_T(struct_cassie_sim)]

cassie_sim_get_jacobian = _libraries['./libcassiemujoco.so'].cassie_sim_get_jacobian
cassie_sim_get_jacobian.restype = None
cassie_sim_get_jacobian.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(ctypes.c_double), ctypes.c_char_p]

cassie_sim_get_jacobian_full = _libraries['./libcassiemujoco.so'].cassie_sim_get_jacobian_full
cassie_sim_get_jacobian_full.restype = None
cassie_sim_get_jacobian_full.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(ctypes.c_double), POINTER_T(ctypes.c_double), ctypes.c_char_p]

cassie_sim_get_jacobian_full_site = _libraries['./libcassiemujoco.so'].cassie_sim_get_jacobian_full_site
cassie_sim_get_jacobian_full_site.restype = None
cassie_sim_get_jacobian_full_site.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(ctypes.c_double), POINTER_T(ctypes.c_double), ctypes.c_char_p]

class struct_c__SA_pd_motor_in_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('torque', ctypes.c_double * 5),
    ('pTarget', ctypes.c_double * 5),
    ('dTarget', ctypes.c_double * 5),
    ('pGain', ctypes.c_double * 5),
    ('dGain', ctypes.c_double * 5),
     ]

pd_motor_in_t = struct_c__SA_pd_motor_in_t
class struct_c__SA_pd_task_in_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('torque', ctypes.c_double * 6),
    ('pTarget', ctypes.c_double * 6),
    ('dTarget', ctypes.c_double * 6),
    ('pGain', ctypes.c_double * 6),
    ('dGain', ctypes.c_double * 6),
     ]

pd_task_in_t = struct_c__SA_pd_task_in_t
class struct_c__SA_pd_leg_in_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('taskPd', pd_task_in_t),
    ('motorPd', pd_motor_in_t),
     ]

pd_leg_in_t = struct_c__SA_pd_leg_in_t
struct_c__SA_pd_in_t._pack_ = True # source:False
struct_c__SA_pd_in_t._fields_ = [
    ('leftLeg', pd_leg_in_t),
    ('rightLeg', pd_leg_in_t),
    ('telemetry', ctypes.c_double * 9),
]

pd_in_t = struct_c__SA_pd_in_t
pack_pd_in_t = _libraries['./libcassiemujoco.so'].pack_pd_in_t
pack_pd_in_t.restype = None
pack_pd_in_t.argtypes = [POINTER_T(struct_c__SA_pd_in_t), POINTER_T(ctypes.c_ubyte)]
unpack_pd_in_t = _libraries['./libcassiemujoco.so'].unpack_pd_in_t
unpack_pd_in_t.restype = None
unpack_pd_in_t.argtypes = [POINTER_T(ctypes.c_ubyte), POINTER_T(struct_c__SA_pd_in_t)]
class struct_PdInput(ctypes.Structure):
    pass

pd_input_t = struct_PdInput
pd_input_alloc = _libraries['./libcassiemujoco.so'].pd_input_alloc
pd_input_alloc.restype = POINTER_T(struct_PdInput)
pd_input_alloc.argtypes = []
pd_input_copy = _libraries['./libcassiemujoco.so'].pd_input_copy
pd_input_copy.restype = None
pd_input_copy.argtypes = [POINTER_T(struct_PdInput), POINTER_T(struct_PdInput)]
pd_input_free = _libraries['./libcassiemujoco.so'].pd_input_free
pd_input_free.restype = None
pd_input_free.argtypes = [POINTER_T(struct_PdInput)]
pd_input_setup = _libraries['./libcassiemujoco.so'].pd_input_setup
pd_input_setup.restype = None
pd_input_setup.argtypes = [POINTER_T(struct_PdInput)]
pd_input_step = _libraries['./libcassiemujoco.so'].pd_input_step
pd_input_step.restype = None
pd_input_step.argtypes = [POINTER_T(struct_PdInput), POINTER_T(struct_c__SA_pd_in_t), POINTER_T(struct_c__SA_cassie_out_t), POINTER_T(struct_c__SA_cassie_user_in_t)]
class struct_c__SA_state_battery_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('stateOfCharge', ctypes.c_double),
    ('current', ctypes.c_double),
     ]

state_battery_out_t = struct_c__SA_state_battery_out_t
class struct_c__SA_state_foot_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('position', ctypes.c_double * 3),
    ('orientation', ctypes.c_double * 4),
    ('footRotationalVelocity', ctypes.c_double * 3),
    ('footTranslationalVelocity', ctypes.c_double * 3),
    ('toeForce', ctypes.c_double * 3),
    ('heelForce', ctypes.c_double * 3),
     ]

state_foot_out_t = struct_c__SA_state_foot_out_t
class struct_c__SA_state_joint_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('position', ctypes.c_double * 6),
    ('velocity', ctypes.c_double * 6),
     ]

state_joint_out_t = struct_c__SA_state_joint_out_t
class struct_c__SA_state_motor_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('position', ctypes.c_double * 10),
    ('velocity', ctypes.c_double * 10),
    ('torque', ctypes.c_double * 10),
     ]

state_motor_out_t = struct_c__SA_state_motor_out_t
class struct_c__SA_state_pelvis_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('position', ctypes.c_double * 3),
    ('orientation', ctypes.c_double * 4),
    ('rotationalVelocity', ctypes.c_double * 3),
    ('translationalVelocity', ctypes.c_double * 3),
    ('translationalAcceleration', ctypes.c_double * 3),
    ('externalMoment', ctypes.c_double * 3),
    ('externalForce', ctypes.c_double * 3),
     ]

state_pelvis_out_t = struct_c__SA_state_pelvis_out_t
class struct_c__SA_state_radio_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('channel', ctypes.c_double * 16),
    ('signalGood', ctypes.c_bool),
    ('PADDING_0', ctypes.c_ubyte * 7),
     ]

state_radio_out_t = struct_c__SA_state_radio_out_t
class struct_c__SA_state_terrain_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('height', ctypes.c_double),
    ('slope', ctypes.c_double * 2),
     ]

state_terrain_out_t = struct_c__SA_state_terrain_out_t
struct_c__SA_state_out_t._pack_ = True # source:False
struct_c__SA_state_out_t._fields_ = [
    ('pelvis', state_pelvis_out_t),
    ('leftFoot', state_foot_out_t),
    ('rightFoot', state_foot_out_t),
    ('terrain', state_terrain_out_t),
    ('motor', state_motor_out_t),
    ('joint', state_joint_out_t),
    ('radio', state_radio_out_t),
    ('battery', state_battery_out_t),
]

state_out_t = struct_c__SA_state_out_t
pack_state_out_t = _libraries['./libcassiemujoco.so'].pack_state_out_t
pack_state_out_t.restype = None
pack_state_out_t.argtypes = [POINTER_T(struct_c__SA_state_out_t), POINTER_T(ctypes.c_ubyte)]
unpack_state_out_t = _libraries['./libcassiemujoco.so'].unpack_state_out_t
unpack_state_out_t.restype = None
unpack_state_out_t.argtypes = [POINTER_T(ctypes.c_ubyte), POINTER_T(struct_c__SA_state_out_t)]
class struct_StateOutput(ctypes.Structure):
    pass

state_output_t = struct_StateOutput
state_output_alloc = _libraries['./libcassiemujoco.so'].state_output_alloc
state_output_alloc.restype = POINTER_T(struct_StateOutput)
state_output_alloc.argtypes = []
state_output_copy = _libraries['./libcassiemujoco.so'].state_output_copy
state_output_copy.restype = None
state_output_copy.argtypes = [POINTER_T(struct_StateOutput), POINTER_T(struct_StateOutput)]
state_output_free = _libraries['./libcassiemujoco.so'].state_output_free
state_output_free.restype = None
state_output_free.argtypes = [POINTER_T(struct_StateOutput)]
state_output_setup = _libraries['./libcassiemujoco.so'].state_output_setup
state_output_setup.restype = None
state_output_setup.argtypes = [POINTER_T(struct_StateOutput)]
state_output_step = _libraries['./libcassiemujoco.so'].state_output_step
state_output_step.restype = None
state_output_step.argtypes = [POINTER_T(struct_StateOutput), POINTER_T(struct_c__SA_cassie_out_t), POINTER_T(struct_c__SA_state_out_t)]
class struct_c__SA_packet_header_info_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('seq_num_out', ctypes.c_char),
    ('seq_num_in_last', ctypes.c_char),
    ('delay', ctypes.c_char),
    ('seq_num_in_diff', ctypes.c_char),
     ]

packet_header_info_t = struct_c__SA_packet_header_info_t
process_packet_header = _libraries['./libcassiemujoco.so'].process_packet_header
process_packet_header.restype = None
process_packet_header.argtypes = [POINTER_T(struct_c__SA_packet_header_info_t), POINTER_T(ctypes.c_ubyte), POINTER_T(ctypes.c_ubyte)]
udp_init_host = _libraries['./libcassiemujoco.so'].udp_init_host
udp_init_host.restype = ctypes.c_int32
udp_init_host.argtypes = [POINTER_T(ctypes.c_char), POINTER_T(ctypes.c_char)]
udp_init_client = _libraries['./libcassiemujoco.so'].udp_init_client
udp_init_client.restype = ctypes.c_int32
udp_init_client.argtypes = [POINTER_T(ctypes.c_char), POINTER_T(ctypes.c_char), POINTER_T(ctypes.c_char), POINTER_T(ctypes.c_char)]
udp_close = _libraries['./libcassiemujoco.so'].udp_close
udp_close.restype = None
udp_close.argtypes = [ctypes.c_int32]
get_newest_packet = _libraries['./libcassiemujoco.so'].get_newest_packet
get_newest_packet.restype = ssize_t
get_newest_packet.argtypes = [ctypes.c_int32, POINTER_T(None), size_t, POINTER_T(struct_sockaddr), POINTER_T(ctypes.c_uint32)]
wait_for_packet = _libraries['./libcassiemujoco.so'].wait_for_packet
wait_for_packet.restype = ssize_t
wait_for_packet.argtypes = [ctypes.c_int32, POINTER_T(None), size_t, POINTER_T(struct_sockaddr), POINTER_T(ctypes.c_uint32)]
send_packet = _libraries['./libcassiemujoco.so'].send_packet
send_packet.restype = ssize_t
send_packet.argtypes = [ctypes.c_int32, POINTER_T(None), size_t, POINTER_T(struct_sockaddr), socklen_t]

__all__ = \
    ['cassie_pelvis_in_t', 'struct_StateOutput', 'cassie_state_t',
    'cassie_sim_check_self_collision', 'cassie_vis_free',
    'cassie_in_t', 'state_terrain_out_t', 'struct_c__SA_pd_leg_in_t',
    'cassie_state_free', 'struct_c__SA_state_battery_out_t',
    'elmo_in_t', 'state_joint_out_t', 'send_packet',
    'cassie_pelvis_out_t', 'cassie_cleanup',
    'struct_c__SA_state_radio_out_t', 'cassie_vis_valid',
    'pd_input_setup', 'pd_leg_in_t', 'cassie_mujoco_init',
    'cassie_state_copy', 'cassie_core_sim_setup', 'battery_out_t',
    'cassie_sim_hold', 'struct_CassieCoreSim', 'cassie_core_sim_step',
    'pack_cassie_out_t', 'cassie_out_t', 'radio_in_t',
    'unpack_cassie_out_t', 'struct_c__SA_pd_task_in_t',
    'struct_PdInput', 'udp_init_client', 'pd_motor_in_t',
    'cassie_sim_t', 'cassie_core_sim_alloc', 'get_newest_packet',
    'size_t', 'struct_c__SA_vectornav_out_t',
    'struct_c__SA_pd_motor_in_t', 'cassie_get_state',
    'state_battery_out_t', 'struct_c__SA_state_pelvis_out_t',
    'cassie_state_qpos', 'cassie_state_qvel', 'state_radio_out_t',
    'struct_c__SA_pd_in_t', 'udp_close', 'state_output_free',
    'cassie_core_sim_free', 'pd_task_in_t', 'packet_header_info_t',
    'pd_in_t', 'struct_cassie_vis', 'struct_c__SA_elmo_out_t',
    'pack_pd_in_t', 'struct_c__SA_radio_out_t', 'pd_input_alloc',
    'DiagnosticCodes', 'unpack_state_out_t', 'target_pc_out_t',
    'cassie_sim_duplicate', 'cassie_state_alloc', 'cassie_sim_init',
    'struct_c__SA_cassie_user_in_t', 'struct_c__SA_radio_in_t',
    'socklen_t', 'cassie_vis_init', 'state_out_t',
    'struct_c__SA_cassie_in_t', 'pd_input_free', 'state_output_alloc',
    'struct_c__SA_cassie_leg_out_t',
    'struct_c__SA_cassie_pelvis_in_t', 'unpack_pd_in_t',
    'cassie_user_in_t', 'cassie_sim_clear_forces', 'cassie_vis_t',
    'struct_c__SA_target_pc_out_t', 'pd_input_step',
    'cassie_set_state', 'struct_c__SA_battery_out_t',
    'vectornav_out_t', 'struct_c__SA_packet_header_info_t',
    'cassie_sim_step_pd', 'struct_sockaddr', 'cassie_vis_draw','cassie_integrate_pos',
    'cassie_core_sim_copy', 'unpack_cassie_in_t', 'struct_cassie_sim',
    'unpack_cassie_user_in_t', 'cassie_sim_step', 'udp_init_host',
    'state_motor_out_t', 'cassie_core_sim_t', 'pack_state_out_t',
    'cassie_sim_mjdata', 'state_output_setup', 'cassie_sim_mjmodel',
    'state_foot_out_t', 'state_output_t', 'cassie_sim_time',
    'cassie_sim_step_ethercat', 'cassie_sim_check_obstacle_collision',
    'elmo_out_t', 'pack_cassie_in_t', 'cassie_sim_apply_force','cassie_sim_full_reset',
    'cassie_leg_out_t', 'wait_for_packet',
    'struct_c__SA_cassie_leg_in_t', 'struct_c__SA_state_joint_out_t',
    'process_packet_header', 'cassie_sim_release', 'cassie_sim_foot_forces', 
    'cassie_sim_foot_positions', 'cassie_sim_foot_velocities', 'struct_c__SA_state_foot_out_t',
    'cassie_sim_cm_position','cassie_sim_centroid_inertia',
    'cassie_sim_cm_velocity','cassie_sim_angular_momentum',
    'cassie_sim_full_mass_matrix','cassie_sim_minimal_mass_matrix',
    'pd_input_t', 'pack_cassie_user_in_t', 'cassie_state_duplicate',
    'state_pelvis_out_t', 'struct_c__SA_state_terrain_out_t',
    'cassie_sim_free', 'cassie_sim_xpos', 'cassie_sim_xquat', 'ssize_t', 'state_output_copy',
    'cassie_sim_radio', 'cassie_vis_close', 'cassie_vis_paused', 'radio_out_t',
    'state_output_step', 'struct_c__SA_state_motor_out_t',
    'struct_cassie_state', 'cassie_state_time', 'cassie_sim_qvel',
    'cassie_sim_qpos', 'cassie_sim_qacc', 'struct_c__SA_elmo_in_t', 'cassie_joint_out_t',
    'cassie_leg_in_t', 'struct_c__SA_cassie_joint_out_t',
    'struct_c__SA_state_out_t', 'struct_c__SA_cassie_pelvis_out_t',
    'pd_input_copy', 'cassie_sim_copy', 'struct_c__SA_cassie_out_t',
    'cassie_sim_dof_damping', 'cassie_sim_set_dof_damping',
    'cassie_sim_body_mass', 'cassie_sim_set_body_mass',
    'cassie_sim_loop_constraint_info',
    'cassie_sim_body_ipos', 'cassie_sim_set_body_ipos',
    'cassie_sim_geom_friction', 'cassie_sim_set_geom_friction',
    'cassie_sim_set_const', 
    'cassie_sim_geom_rgba', 'cassie_sim_geom_name_rgba', 'cassie_sim_set_geom_rgba', 'cassie_sim_set_geom_name_rgba',
    'cassie_sim_geom_quat', 'cassie_sim_geom_name_quat', 'cassie_sim_set_geom_quat', 'cassie_sim_set_geom_name_quat',
    'cassie_sim_geom_pos', 'cassie_sim_geom_name_pos', 'cassie_sim_set_geom_pos', 'cassie_sim_set_geom_name_pos',
    'cassie_sim_geom_size', 'cassie_sim_geom_name_size', 'cassie_sim_set_geom_size', 'cassie_sim_set_geom_name_size',
    'cassie_sim_set_geom_name_friction', 'cassie_reload_xml', 'cassie_vis_apply_force',
    'cassie_vis_add_marker', 'cassie_vis_remove_marker', 'cassie_vis_clear_markers',
    'cassie_vis_update_marker_pos', 'cassie_vis_update_marker_size', 'cassie_vis_update_marker_rgba', 'cassie_vis_update_marker_orient',
    'cassie_sim_foot_quat', 'cassie_sim_body_vel', 'cassie_sim_set_body_name_mass',
    'cassie_sim_get_hfield_nrow', 'cassie_sim_get_hfield_ncol', 'cassie_sim_get_nhfielddata',
    'cassie_sim_get_hfield_size', 'cassie_sim_set_hfield_size', 'cassie_sim_hfielddata', 'cassie_sim_set_hfielddata',
    'cassie_sim_foot_quat', 'cassie_sim_body_vel', 'cassie_sim_set_body_name_mass', 'cassie_vis_set_cam', 
    'cassie_sim_joint_filter', 'cassie_sim_drive_filter', 'cassie_sim_set_joint_filter', 'cassie_sim_set_drive_filter',
    'cassie_sim_get_joint_filter',  'cassie_sim_get_drive_filter', 
    'cassie_sim_torque_delay', 'cassie_sim_set_torque_delay', 'drive_filter_t', 'joint_filter_t',
    'cassie_sim_params', 'cassie_sim_nv', 'cassie_sim_nbody', 'cassie_sim_nq', 'cassie_sim_ngeom',
    'cassie_vis_record_frame', 'cassie_vis_init_recording', 'cassie_vis_close_recording', 'cassie_vis_window_resize', 'cassie_vis_attach_cam',
    'cassie_vis_draw_depth', 'cassie_vis_get_depth_size', 'cassie_vis_init_depth', 'cassie_vis_attach_cam', 'cassie_vis_remakeSceneCon', 'cassie_vis_full_reset',
    'cassie_sim_get_jacobian', 'cassie_sim_get_jacobian_full', 'cassie_sim_get_jacobian_full_site', 'cassie_sim_get_body_name_pos', 'cassie_sim_set_body_name_pos',
    'cassie_sim_heeltoe_forces', 'cassie_vis_extent', 'cassie_vis_znear', 'cassie_vis_zfar']


