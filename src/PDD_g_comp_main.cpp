#include <yarp/os/all.h>
#include <GYM/generic_module.hpp>
#include <cstdlib>

#include "PDD_g_comp_module.hpp"

// default module period
#define MODULE_PERIOD 1000 //[millisec]

int main(int argc, char* argv[])
{
    // yarp network declaration and check
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()){
        std::cerr <<"yarpserver not running - run yarpserver"<< std::endl;
        exit(EXIT_FAILURE);
    }
    // yarp network initialization
    yarp.init();

    // create rf
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    // set PDD_g_comp_initial_config.ini as default
    // to specify another config file, run with this arg: --from your_config_file.ini 
    rf.setDefaultConfigFile( "PDD_g_comp_initial_config.ini" ); 
    rf.setDefaultContext( "PDD_g_comp" );  
    rf.configure(argc, argv);
    // create my module
    PDD_g_comp_module PDD_g_comp_mod = PDD_g_comp_module( argc, argv, "PDD_g_comp", MODULE_PERIOD, rf );
        
    // yarp network deinitialization
    yarp.fini();
    
    // run the module
    PDD_g_comp_mod.runModule( rf );
    
    exit(EXIT_SUCCESS);
}
