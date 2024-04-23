import lcm
from obslcm import observ_t
import torch
from policy_loader import WeightPolicy




def load_policy():
    model = WeightPolicy()
    print("Policy loaded !!!!!!!!")

def my_handler(channel, data):
    msg = observ_t.decode(data)
    load_policy()
    print("Received message on channel \"%s\"" % channel)
    print("   base_lin_vel   = %s" % str(msg.base_lin_vel))
    print("   base_ang_vel    = %s" % str(msg.base_ang_vel))
    print("   projected_gravity = %s" % str(msg.projected_gravity))
    print("   commands %s" % str(msg.commands))
    print("   dof_pos        = '%s'" % str(msg.dof_pos))
    print("   dof_vel        = '%s'" % str(msg.dof_vel))
    print("   _action    = '%s'" % str(msg._action))

    print("")
    


    
msg = observ_t()
lc = lcm.LCM()
subscription = lc.subscribe("OBSERVATION", my_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass
