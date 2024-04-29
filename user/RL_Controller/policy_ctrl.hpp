#ifndef JPOS_CONTROLLER
#define JPOS_CONTROLLER

#define UNUSED(x) (void)(x)


#include <RobotController.h>
//#include <RobotRunner.h>
#include "RaspberryPiParameters.h"
#include <Controllers/FootSwingTrajectory.h>
#include <Controllers/StateEstimatorContainer.h>
#include <Dynamics/Quadruped.h>

#include <unistd.h>
#include <lcm/lcm-cpp.hpp>
#include "action_t.hpp"
#include "observ_t.hpp"

extern lcm::LCM policy_lcm;

extern float tau_policy[12];


class policy_Controller:public RobotController{
  public:
    policy_Controller():RobotController(){ }

    virtual void initializeController();
    virtual void runController();
    virtual void updateVisualization(){ }
    //void updateRaspCommand();
    //void updateGamepadCommand();
    //void legDetection();
    float leg_Interpolation(const size_t & curr_iter, size_t max_iter,
                          const float & ini, const float & fin);
    virtual ControlParameters* getUserControlParameters() {
      return &userParameters;
    }
  protected:
    // float rx_ini[12], q_home[12], q_now[12];
    // float q_ini[12] = {-0.6435f, -0.3187f, -0.3709f,
    //                    -0.7594f, -0.2840f, -0.1709f,
    //                    -0.8087f, -0.4086f, -0.9566f,
    //                    -0.5681f,  0.0146f -1, -0.7188f};
    Vec3<float> _pfoot_ini;
    Vec3<float> zero_vec3;
    policyUserParameters userParameters;
    Mat3<float> kpMat;
    Mat3<float> kdMat;
    //float tau_policy[12];

  private:
    StateEstimatorContainer<float>* _stateEstimator;

    obslcm::observ_t OBSERVATION;

    

    
    long j_iter = 0;
    int motion_iter = 0;

    Vec4<int> leg_enable;
    //Vec3<float> desGamepadCommand;

};

class Handler {
    public:
        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan,
                const obslcm::action_t* msg)
        {
          UNUSED(rbuf);
          UNUSED(chan);
          //printf("Receiving message on channel \"%s\":\n", chan.c_str());
          for(int i(0); i < 12; i++) {
            //printf("in action lcm");
            tau_policy[i] = msg->tau[i];
            //printf("%f \n", tau_policy[i]);
              //printf("Data for motor number %d : %f , %f \n", i, q[i], qd[i]);
            
          }  
        }
};

#endif
