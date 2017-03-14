/*
 * Copyright (c) 2017, <copyright holder> <email>
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

#ifndef FORCE_ESTIMATE_H
#define FORCE_ESTIMATE_H

#include <iostream>
#include <string.h>
#include <fstream>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>

namespace ft_perception
{
    class FTEstimation
    {
      private:
          std::string hand_name_;
          std::string robot_name_;
          std::string whole_body_dynamics_module_name_;
          yarp::sig::Vector *wrench_estimate_;
          
          yarp::os::BufferedPort<yarp::sig::Vector> *end_effector_wrench_input_port_;
          
      public:
          bool log_data_;
          std::string data_directory_;
          std::ofstream file_name_;
          
          yarp::os::RpcClient ft_estimation_rpc;
          FTEstimation()
          {
              std::cout << "FT estimation default constructor" << std::endl;
          }
          
          FTEstimation(std::string,std::string,std::string);
          bool connectToWDB();
          bool getWrench();
          void displayWrench();
          bool wbdResetOffset();
          bool wbdCalib();
    };
}

#endif // FORCE_ESTIMATE_H
