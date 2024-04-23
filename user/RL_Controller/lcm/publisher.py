import lcm
from obslcm import observ_t
from RL_Controller import _legController

msg = observ_t()

for leg in range(1,5):
    for joint in range(1,4):
        msg.base_lin_vel = _legController.datas[leg].v
        msg.base_ang_vel = _legController.datas[leg].v
        msg.projected_gravity = (0, 0, 9.8)
        msg.commands = _legController._stateEstimatorContainer
        msg.dof_pos = _legController.datas[leg].q[joint]
        msg.dof_vel =_legController.datas[leg].qd[joint]
        msg._action = _legController._stateEstimate


lc = lcm.LCM()
lc.publish("OBSERVATION", msg.encode())














        # msg.q = _legController.datas[leg].q[joint]
        # msg.qdot = _legController.datas[leg].qd[joint]
        # msg.orientation = 
        # msg.pre_action = _legController.datas[leg].tau[joint]
        # msg.cmd_vel = 


# observations = np.concatenate((base_lin_vel, 
#                                        base_ang_vel, 
#                                        projected_gravity, 
#                                        commands, 
#                                        dof_pos, 
#                                        dof_vel, 
#                                        _actions))

# msg.q = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
# msg.qdot = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
# msg.orientation = (1,2,3,4,5,6)
# msg.pre_action = (6.0,6.0,6.0,6.0,6.0,6.0,6.0,6.0,6.0,6.0,6.0,6.0)
# msg.cmd_vel = (1.0, 0.0, 0.0, 0.0, 0.0, 0.0 )