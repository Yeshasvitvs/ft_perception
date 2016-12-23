#include <iostream>
#include <ft_module.h>

int main(int argc, char **argv) {
    
    yarp::os::Network yarp;
    
    FTModule ftModule;
    yarp::os::ResourceFinder rf;
    //rf.setVerbose(true);
    rf.configure(argc,argv);
    
    ConstString robotName = rf.find("robot").asString();
    std::cout << robotName << std::endl;
    if(!ftModule.runModule(rf))
    {
        std::cerr << "FTModule failed to start!" << std::endl;
        return 1;
    }
    return 0;
}
