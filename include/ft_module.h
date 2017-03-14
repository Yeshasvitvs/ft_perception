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
#include <fstream>
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
        yarp::os::RpcServer rpc_port;
        int count;
        std::string robotName;
        std::string handName;
        std::string WBDName;
        
        ft_perception::FTEstimation *ftEstimate;
    public:
        double getPeriod();
        bool updateModule();
        
        bool respond(yarp::os::Bottle& command,yarp::os::Bottle& reply)
        {
           std::string cmd = command.get(0).asString();
           std::string cmd1 = command.get(1).asString();
           
           std::cout << "received command : " << cmd << " " << cmd1 << std::endl;
           if (cmd == "quit") return false; 
           else
           {
               //set log data flag
               if(cmd == "log")
               {
                   if(cmd1 == "start")
                   {
                       if(ftEstimate->log_data_ == true)
                       {
                           reply.addString("Data Logging: [INPROGRESS]");
                       }
                       else
                       {
                           if(command.size() < 3)
                               reply.addString("Incorrect arguments! Correct Usage : log start filename");
                           else
                           {
                               std::string cmd2 = command.get(2).asString();
                               cmd2 = ftEstimate->data_directory_ + "/" + cmd2;
                               std::cout << "filename : " << cmd2 << std::endl;
                               ftEstimate->log_data_ = true;
                               ftEstimate->file_name_.open(cmd2);
                               std::string dummy = "Data Logging : [START] to " + cmd2;
                            }
                       }
                   }
                   if(cmd1 == "stop")
                   {
                       if(ftEstimate->log_data_ == false)
                           reply.addString("Data Logging : [NOT IN PROGRESS]");
                       else
                       {
                           ftEstimate->log_data_ = false;
                           ftEstimate->file_name_.close();
                           reply.addString("Data Logging : [STOP");
                       }
                   }
               }
           }
            return true;
            
        }
        
        bool configure(yarp::os::ResourceFinder &rf)
        {
            count=0;
            if(!yarp::os::Network::initialized())
            {
                yarp::os::Network::init();
            }
            if(rpc_port.open("/ftModule/rpc:i"))
            {
                std::cout << "Opened the port " << rpc_port.getName() << std::endl;
                attach(rpc_port);
            }
            robotName = rf.find("robot").asString();         
            handName = rf.find("hand").asString();
            WBDName = rf.find("wbdModule").asString();
            
            std::cout << "Initializing force estimation object from ft module" << std::endl;
            ftEstimate = new ft_perception::FTEstimation(this->robotName,this->handName,this->WBDName);
            
            //ftEstimate->wbdResetOffset();
            //ftEstimate->wbdCalib();
            
            
            return true;
            
        }
        
        bool interruptModule();
        bool close();
        
    };
}

#endif // FT_MODULE_H

