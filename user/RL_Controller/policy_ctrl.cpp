#include "policy_ctrl.hpp"
#include "cppTypes.h"


float tau[12];
float q[12];
float qdot[12];
float pre_action[12];
float orientation[6];
float cmd_vel[12];
float tau_policy[12];


Handler handlerObject;
lcm::LCM policy_lcm("udpm://224.0.55.55:5001?ttl=225");

void policy_Controller::initializeController() {
  // Real Robot
  for(int i(0); i < 12; i++){
    tau[i] = 0;
    q[i] = 0;
    qdot[i] = 0;
    pre_action[i] = 0;
  }
  for(int i(0); i < 6; i++){
    orientation[i] = 0;
    cmd_vel[i] = 0;
  }

  // Simulation
  _legController->_maxTorque = userParameters.max_tau;
  _legController->_legsEnabled = true;

  // Both
  leg_enable << 0, 0, 0, 0;
  // desGamepadCommand << 0.0, 0.0, 0.0;

    if(userParameters.calibrate > 0.4) {  // This param is only used in TI board in Cheetah3
        _legController->_calibrateEncoders = userParameters.calibrate;
    } else {
        if (userParameters.zero > 0.5) {  //　This param is only used in TI board in Cheetah3
            _legController->_zeroEncoders = true;
        } else {
            _legController->_zeroEncoders = false;
        }
    }

  //q_ini[] = {-0.6435f, -0.052f, -0.4289f, -0.7594f, -0.5449f, -0.9682f, -0.8087f, -0.6638f, -0.2434f, -0.5681f, -0.7652f, -0.0636f};
  
  printf("I'm doing OK!\n");
}



void policy_Controller::runController() {
  ++j_iter;
  //legDetection();

  // Real Robot
  //updateRaspCommand();
  for(int i(0); i < 4; i++) {
    OBSERVATION.dof_pos[3*i] = _legController->datas[i].q[0];
    OBSERVATION.dof_pos[3*i+1] = _legController->datas[i].q[1];
    OBSERVATION.dof_pos[3*i+2] = _legController->datas[i].q[2];

    OBSERVATION.dof_vel[3*i] = _legController->datas[i].qd[0];
    OBSERVATION.dof_vel[3*i+1] = _legController->datas[i].qd[1];
    OBSERVATION.dof_vel[3*i+2] = _legController->datas[i].qd[2];

    OBSERVATION.action[3*i] = _legController->datas[i].tauEstimate[0];
    OBSERVATION.action[3*i+1] = _legController->datas[i].tauEstimate[1];
    OBSERVATION.action[3*i+2] = _legController->datas[i].tauEstimate[2];  
  }

  for (int i(0);i<3; i++){
    OBSERVATION.base_lin_vel[i] = 0.0;
    OBSERVATION.base_ang_vel[i] = 0.0;
    OBSERVATION.gravity[0] = 0.0;
    OBSERVATION.gravity[1] = 0.0;
    OBSERVATION.gravity[2] = -9.8;
    OBSERVATION.commands[i] = 0.0;
  }
  //printf("publishing");
  policy_lcm.publish("OBSERVATION", &OBSERVATION);

  policy_lcm.subscribe("ACTION", &Handler::handleMessage, &handlerObject);
  for(int i(0); i < 4; i++){
    _legController->commands[i].tauFeedForward = {tau_policy[3*i], tau_policy[3*i+1], tau_policy[3*i+2]};
  }
  //printf("%f \n",tau_policy[11]);

  
  policy_lcm.handle();
  // Simulation
  kpMat << userParameters.kp, 0, 0, 0,  userParameters.kp, 0, 0, 0,  userParameters.kp;
  kdMat << userParameters.kd, 0, 0, 0, userParameters.kd, 0, 0, 0, userParameters.kd;

  for(int legIndex(0); legIndex < 4; legIndex++) {
    _legController->commands[legIndex].kpJoint = kpMat;
    _legController->commands[legIndex].kdJoint = kdMat;
  }

  }

  

  



