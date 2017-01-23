/*
 * Copyright (c) 2016, <copyright holder> <email>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#ifndef FT_MODULE_H
#define FT_MODULE_H

#include <iostream>
#include <iomanip>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include <ft_estimate.h>

using namespace std;
using namespace yarp::os;

namespace ft_perception
{
    class FTModule:public yarp::os::RFModule
    {
        yarp::os::Port handlePort;
        int count;
        std::string robotName;
        std::string handName;
        std::string WBDName;
        
        ft_perception::FTEstimation *ftEstimate_;
    public:
        double getPeriod();
        bool updateModule();
        
        bool respond(yarp::os::Bottle& command,yarp::os::Bottle& reply)
        {
            //Simple ROS kinda service reply
            std::cout << "Received something" << std::endl;
            if(command.get(0).asString()=="quit") return false;
            else reply = command;
            return true;
            
        }
        
        bool configure(yarp::os::ResourceFinder &rf)
        {
            count=0;
            //Attaching a port to this module, the received messages can be used in respond method
            handlePort.open("/ftModule");
            attach(handlePort);
            robotName = rf.find("robot").asString();         
            handName = rf.find("hand").asString();
            WBDName = rf.find("wbdModule").asString();
            
            std::cout << "Initializing force estimation object from ft module" << std::endl;
            ftEstimate_ = new ft_perception::FTEstimation(this->robotName,this->handName,this->WBDName);
            
            //ftEstimate_->wbdResetOffset();
            //ftEstimate_->wbdCalib();
            
            
            return true;
            
        }
        
        bool interruptModule();
        bool close();
        
    };
}

#endif // FT_MODULE_H

