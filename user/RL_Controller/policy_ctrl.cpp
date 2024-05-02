#include "policy_ctrl.hpp"
#include "cppTypes.h"


float tau[12];
float q[12];
float qdot[12];
float pre_action[12];
float orientation[6];
float cmd_vel[12];
float tau_policy[12];
float q_now[12];
float qdot_now[12];


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

  
  // std::cout<<j_iter;

  

  // }
  //legDetection();

  // Real Robot
  //updateRaspCommand();
  


  //printf("publishing");
  //std::cout<< _legController->datas->tauEstimate[0]<<" "<< _legController->datas->tauEstimate[1]<<"  "<< _legController->datas->tauEstimate[2] <<"\n";
  // std::cout<<"+++++++++++++++++++++++++++++++++++++++++++++++++++++++++="<<"\n";
  // std::cout<<OBSERVATION.base_lin_vel[0]<<"  "<<OBSERVATION.base_lin_vel[1]<<"  "<<OBSERVATION.base_lin_vel[2]<<"  "<<"\n";
  float q_home[12] = {0.1,-0.8,1.7,0.1,-0.8,1.7,0.1,-0.8,1.7,0.1,-0.8,1.7};
  float q_des[12];

  if (j_iter<=16000){
    kpMat << userParameters.kp, 0, 0, 0,  userParameters.kp, 0, 0, 0,  userParameters.kp;
  kdMat << userParameters.kd, 0, 0, 0, userParameters.kd, 0, 0, 0, userParameters.kd;

    for(int legIndex(0); legIndex < 4; legIndex++) {
      _legController->commands[legIndex].kpJoint = kpMat;
      _legController->commands[legIndex].kdJoint = kdMat;
    }
    // std::cout<<"hereeeeeee\n";
    for (int i(0); i<4 ; i++){
      q_now[3*i] = _legController->datas[i].q[0];
      q_now[3*i+1] =_legController->datas[i].q[1];
      q_now[3*i+2] =_legController->datas[i].q[2];
    }
    
    for(int i(0); i < 12; i++) {
          std::cout<<j_iter<<"\n";
          q_des[i] = leg_Interpolation(j_iter, 16000, q_now[i], q_home[i]);
          // std::cout<<q_des[2]<<"\n";
        }

        _legController->commands[0].qDes = {q_des[0],q_des[1],q_des[2]};
        _legController->commands[1].qDes = {q_des[3],q_des[4],q_des[5]};
        _legController->commands[2].qDes = {q_des[6],q_des[7],q_des[8]};
        _legController->commands[3].qDes = {q_des[9],q_des[10],q_des[11]};
        
        }
  if (j_iter>16000){
    
    

    if ((OBSERVATION.dof_pos[2]>2.697) && (OBSERVATION.dof_pos[5]>2.697) && (OBSERVATION.dof_pos[8]>2.697) && (OBSERVATION.dof_pos[11]>2.697)){
      kpMat << userParameters.kp, 0, 0, 0,  userParameters.kp, 0, 0, 0,  userParameters.kp;
      kdMat << userParameters.kd, 0, 0, 0, userParameters.kd, 0, 0, 0, userParameters.kd;
      _legController->commands[0].qDes[2] = 2.697;
      _legController->commands[1].qDes[2] = 2.697;
      _legController->commands[2].qDes[2] = 2.697;
      _legController->commands[3].qDes[2] = 2.697;
      std::cout<< "calf now:  "<<OBSERVATION.dof_pos[2] <<"  "<<OBSERVATION.dof_pos[5]<< "   "<< OBSERVATION.dof_pos[11]<< "\n";
      std::cout<<"danger zone calf"<<"\n";
      // repair = true;
    }




    if ((OBSERVATION.dof_pos[2]<0.9) && (OBSERVATION.dof_pos[5]<0.9) && (OBSERVATION.dof_pos[8]<0.9) && (OBSERVATION.dof_pos[11]<0.9)){
      kpMat << userParameters.kp, 0, 0, 0,  userParameters.kp, 0, 0, 0,  userParameters.kp;
      kdMat << userParameters.kd, 0, 0, 0, userParameters.kd, 0, 0, 0, userParameters.kd;
      _legController->commands[0].qDes[2] = 0.9;
      _legController->commands[1].qDes[2] = 0.9;
      _legController->commands[2].qDes[2] = 0.9;
      _legController->commands[3].qDes[2] = 0.9;
      std::cout<<"danger zone calf"<<"\n";

    }

    if ((OBSERVATION.dof_pos[0]>0.8) && (OBSERVATION.dof_pos[3]>0.8) && (OBSERVATION.dof_pos[6]>0.8) && (OBSERVATION.dof_pos[9]>0.8)){
      kpMat << userParameters.kp, 0, 0, 0,  userParameters.kp, 0, 0, 0,  userParameters.kp;
      kdMat << userParameters.kd, 0, 0, 0, userParameters.kd, 0, 0, 0, userParameters.kd;
      _legController->commands[0].qDes[0] = 0.8;
      _legController->commands[1].qDes[0] = 0.8;
      _legController->commands[2].qDes[0] = 0.8;
      _legController->commands[3].qDes[0] = 0.8;
      std::cout<<"danger zone hip"<<"\n";

    }

    if ((OBSERVATION.dof_pos[0]<-0.8) && (OBSERVATION.dof_pos[3]<-0.8) && (OBSERVATION.dof_pos[6]<-0.8) && (OBSERVATION.dof_pos[9]<-0.8)){
      kpMat << userParameters.kp, 0, 0, 0,  userParameters.kp, 0, 0, 0,  userParameters.kp;
      kdMat << userParameters.kd, 0, 0, 0, userParameters.kd, 0, 0, 0, userParameters.kd;
      _legController->commands[0].qDes[0] = -0.8;
      _legController->commands[1].qDes[0] = -0.8;
      _legController->commands[2].qDes[0] = -0.8;
      _legController->commands[3].qDes[0] = -0.8;
      std::cout<<"danger zone hip"<<"\n";

    }


    if ((OBSERVATION.dof_pos[1]>1.04) && (OBSERVATION.dof_pos[4]>1.04) && (OBSERVATION.dof_pos[7]>1.04) && (OBSERVATION.dof_pos[10]>1.04)){
      kpMat << userParameters.kp, 0, 0, 0,  userParameters.kp, 0, 0, 0,  userParameters.kp;
      kdMat << userParameters.kd, 0, 0, 0, userParameters.kd, 0, 0, 0, userParameters.kd;
      _legController->commands[0].qDes[1] = 1.04;
      _legController->commands[1].qDes[1] = 1.04;
      _legController->commands[2].qDes[1] = 1.04;
      _legController->commands[3].qDes[1] = 1.04;
      std::cout<<"danger zone thigh"<<"\n";

    }

    if ((OBSERVATION.dof_pos[1]<-4.188) && (OBSERVATION.dof_pos[4]<-4.188) && (OBSERVATION.dof_pos[7]<-4.188) && (OBSERVATION.dof_pos[10]<-4.188)){
      kpMat << userParameters.kp, 0, 0, 0,  userParameters.kp, 0, 0, 0,  userParameters.kp;
      kdMat << userParameters.kd, 0, 0, 0, userParameters.kd, 0, 0, 0, userParameters.kd;
      _legController->commands[0].qDes[1] = -4.188;
      _legController->commands[1].qDes[1] = -4.188;
      _legController->commands[2].qDes[1] = -4.188;
      _legController->commands[3].qDes[1] = -4.188;
      std::cout<<"danger zone thigh"<<"\n";

    }




    // std::cout<<OBSERVATION.dof_pos[0]<<"  "<<OBSERVATION.dof_pos[1]<<"  "<<OBSERVATION.dof_pos[2]<<"  "<<"\n";
    // std::cout<<OBSERVATION.dof_pos[3]<<"  "<<OBSERVATION.dof_pos[4]<<"  "<<OBSERVATION.dof_pos[5]<<"  "<<"\n";
    // std::cout<<OBSERVATION.dof_pos[6]<<"  "<<OBSERVATION.dof_pos[7]<<"  "<<OBSERVATION.dof_pos[8]<<"  "<<"\n";
    // std::cout<<OBSERVATION.dof_pos[9]<<"  "<<OBSERVATION.dof_pos[10]<<"  "<<OBSERVATION.dof_pos[11]<<"  "<<"\n";
    // std::cout<<"--------------------------------------------------"<<"\n";
    // std::cout<<OBSERVATION.dof_vel[0]<<"  "<<OBSERVATION.dof_vel[1]<<"  "<<OBSERVATION.dof_vel[2]<<"  "<<"\n";
    // std::cout<<OBSERVATION.dof_vel[3]<<"  "<<OBSERVATION.dof_vel[4]<<"  "<<OBSERVATION.dof_vel[5]<<"  "<<"\n";
    // std::cout<<OBSERVATION.dof_vel[6]<<"  "<<OBSERVATION.dof_vel[7]<<"  "<<OBSERVATION.dof_vel[8]<<"  "<<"\n";
    // std::cout<<OBSERVATION.dof_vel[9]<<"  "<<OBSERVATION.dof_vel[10]<<"  "<<OBSERVATION.dof_vel[11]<<"  "<<"\n";
    // std::cout<<"--------------------------------------------------"<<"\n";
    // std::cout<<OBSERVATION.action[0]<<"  "<<OBSERVATION.action[1]<<"  "<<OBSERVATION.action[2]<<"  "<<"\n";
    // std::cout<<OBSERVATION.action[3]<<"  "<<OBSERVATION.action[4]<<"  "<<OBSERVATION.action[5]<<"  "<<"\n";
    // std::cout<<OBSERVATION.action[6]<<"  "<<OBSERVATION.action[7]<<"  "<<OBSERVATION.action[8]<<"  "<<"\n";
    // std::cout<<OBSERVATION.action[9]<<"  "<<OBSERVATION.action[10]<<"  "<<OBSERVATION.action[11]<<"  "<<"\n";
    // std::cout<<"--------------------------------------------------"<<"\n";
    // std::cout<<OBSERVATION.base_lin_vel[0]<<"  "<<OBSERVATION.base_lin_vel[1]<<"  "<<OBSERVATION.base_lin_vel[2]<<"  "<<"\n";
    // std::cout<<"--------------------------------------------------"<<"\n";
    // std::cout<<OBSERVATION.base_ang_vel[0]<<"  "<<OBSERVATION.base_ang_vel[1]<<"  "<<OBSERVATION.base_ang_vel[2]<<"  "<<"\n";
    // std::cout<<"--------------------------------------------------"<<"\n";
    // std::cout<<OBSERVATION.gravity[0]<<"  "<<OBSERVATION.gravity[1]<<"  "<<OBSERVATION.gravity[2]<<"  "<<"\n";
    // std::cout<<"--------------------------------------------------"<<"\n";
    // std::cout<<OBSERVATION.gravity[0]<<"  "<<OBSERVATION.gravity[1]<<"  "<<OBSERVATION.gravity[2]<<"  "<<"\n";
    // std::cout<<"--------------------------------------------------"<<"\n";
    // std::cout<<OBSERVATION.commands[0]<<"  "<<OBSERVATION.commands[1]<<"  "<<OBSERVATION.commands[2]<<"  "<<"\n";
    

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
      OBSERVATION.base_lin_vel[i] = _stateEstimate->vBody[i];
      OBSERVATION.base_ang_vel[i] = _stateEstimate->omegaBody[i];
      OBSERVATION.gravity[0] = 0.0;
      OBSERVATION.gravity[1] = 0.0;
      OBSERVATION.gravity[2] = -9.8;
      OBSERVATION.commands[i] = 0.0;
    }
    for (int i(0); i<4; i++){
      OBSERVATION.quaternion[i] = _stateEstimate->orientation[i];
    }

    policy_lcm.publish("OBSERVATION", &OBSERVATION);

    policy_lcm.subscribe("ACTION", &Handler::handleMessage, &handlerObject);
    policy_lcm.handle();

    // Simulation
    kpMat << 0, 0, 0, 0,  0 , 0, 0, 0,  0;
    kdMat << 0, 0, 0, 0, 0, 0, 0, 0, 0;

    for(int legIndex(0); legIndex < 4; legIndex++) {
      _legController->commands[legIndex].kpJoint = kpMat;
      _legController->commands[legIndex].kdJoint = kdMat;
    }
  for(int i(0); i < 4; i++){
    if (repair== true){
      repair= true;
    }
    else{
      _legController->commands[i].tauFeedForward = {tau_policy[3*i], tau_policy[3*i+1], tau_policy[3*i+2]};
    }
    
  }
  //printf("%f \n",tau_policy[11]);

  }
  
  }


float policy_Controller::leg_Interpolation(const size_t & curr_iter, size_t max_iter,
                                        const float & ini, const float & fin) {

    float a(0.f);
    float b(1.f);

    // interpolating
    if(curr_iter <= max_iter) {
      b = (float)curr_iter / (float)max_iter;
      a = 1.f - b;
    }

    // compute setpoints
    float inter_pos = a * ini + b * fin;

    // send control commands
    return inter_pos;
}


  



