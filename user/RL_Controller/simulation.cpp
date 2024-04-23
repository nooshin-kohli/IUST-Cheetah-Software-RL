#include <iostream>
#include <chrono>
#include <math.h>

#include <limits>
#include <fstream>
#include <thread>
#include <lcm/lcm-cpp.hpp>
#include "action_t.hpp"
#include "observ_t.hpp"


using namespace std::chrono;
using namespace std;


lcm::LCM lc("udpm://224.0.55.55:5001?ttl=225");
double t_1, t_2;
class Handler
{
    public:
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan,
                const action_t* msg)
        {
            printf("Received message on channel \"%s\":\n", chan.c_str());
            
            t_1 = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
            cout<< "commanding motors"<< endl;

            t_2 = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
            cout << "Delta t:" << t_2 - t_1 << endl;

            observ_t DATA;
            cout<<"publishing"<<endl;
            for(int i=0;i<12;i++){
                DATA.q[i] = 0.0;
                DATA.qdot[i] = 0.0;
                DATA.tau[i] = 0.0;
                DATA.pre_action[i] = 0.0;
                DATA.orientation[i] = 0.0;
                DATA.cmd_vel[i] = 0.0;
            }
            for (int i=0; i<6;i++){
                DATA.orientation[i] = 0.0;
                DATA.cmd_vel[i] = 0.0;
            }

            
            cout<<"end of publishing data"<<endl;
            lc.publish("OBSERVATION", &DATA);

        }
};

int main(int argc, char** argv)
{

    if(!lc.good())
        return 1;

    Handler handlerObject;
    lc.subscribe("ACTION", &Handler::handleMessage, &handlerObject);

    while(0 == lc.handle());

    return 0;
}