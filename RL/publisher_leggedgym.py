# this script is a test for input/output of policy
# legged gym
import lcm
from obslcm import observ_t
from obslcm import action_t
import numpy as np

msg_obs = observ_t()

msg_obs.base_lin_vel = [1.1809e-03, -5.7731e-04,  3.3632e-03]
msg_obs.base_ang_vel = [4.4893e-03, -7.2973e-04, 2.3229e-06]
msg_obs.gravity = [0.0,0.0,-9.8]
msg_obs.commands = [0.0,0.0,0.0]
msg_obs.dof_pos = [-9.7234e-02,  8.0466e-01, -1.7088e+00, 1.0302e-01,  8.0664e-01, -1.7100e+00, -1.5477e-01,  7.7372e-01, -1.7454e+00,  4.5528e-02,  7.7073e-01, -1.7499e+00]
msg_obs.dof_vel = [-1.1306e-03,2.1604e-04, -3.0156e-04, -1.0980e-03, -2.4234e-04,  2.7714e-04,-1.0606e-03,  2.0990e-04, -4.6655e-04, -9.4770e-04, -2.6237e-04,2.8932e-04]
msg_obs.action = [1.4960e+00,  2.3061e+00, -4.3727e+00, -1.3981e+00,3.3454e+00, -5.0455e+00, -2.7279e+01, -1.3159e+01, -2.2634e+01,2.7332e+01, -1.4608e+01, -2.4983e+01]

# observations = [[ 1.1809e-03, -5.7731e-04,  3.3632e-03,///  4.4893e-03, -7.2973e-04,
#           2.3229e-06, /// 0.0000e+00,  0.0000e+00, -9.8000e+00,  ///0.0000e+00,
#           0.0000e+00,  0.0000e+00,/// -9.7234e-02,  8.0466e-01, -1.7088e+00,
#           1.0302e-01,  8.0664e-01, -1.7100e+00, -1.5477e-01,  7.7372e-01,
#          -1.7454e+00,  4.5528e-02,  7.7073e-01, -1.7499e+00,/// -1.1306e-03,
#           2.1604e-04, -3.0156e-04, -1.0980e-03, -2.4234e-04,  2.7714e-04,
#          -1.0606e-03,  2.0990e-04, -4.6655e-04, -9.4770e-04, -2.6237e-04,
#           2.8932e-04,///  1.4960e+00,  2.3061e+00, -4.3727e+00, -1.3981e+00,
#           3.3454e+00, -5.0455e+00, -2.7279e+01, -1.3159e+01, -2.2634e+01,
#           2.7332e+01, -1.4608e+01, -2.4983e+01]]


lc = lcm.LCM() # "udpm://224.0.55.55:5001?ttl=225"
lc.publish("OBSERVATION", msg_obs.encode())

def my_handler(channel, data):
    msg_action = action_t.decode(data)
    print("Received message on channel \"%s\"" % channel)
    print(" actions  = %s" % str(msg_action.tau))
    lc.publish("OBSERVATION", msg_obs.encode())
    print('################################################################################')
   


msg_action = action_t()
lc = lcm.LCM() # "udpm://224.0.55.55:5001?ttl=225"
subscription = lc.subscribe("ACTION", my_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass
