import lcm
from dofstatelcm import dofstate_t

def my_handler(channel, data):
    msg = dofstate_t.decode(data)
    print("Received message on channel \"%s\"" % channel)
    print("   dofstate_pos   = %s" % str(msg.dof_pos))
    print("   dofstate_vel   = %s" % str(msg.dof_vel))
    print('##########################################################################')
    
lc = lcm.LCM()
subscription = lc.subscribe("DOFSTATE", my_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass