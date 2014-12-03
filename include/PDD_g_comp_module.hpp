#ifndef PDD_g_comp_MODULE_HPP_
#define PDD_g_comp_MODULE_HPP_

#include <GYM/generic_module.hpp>

#include "PDD_g_comp_thread.h"

/**
 * @brief _MODULE_PREFIX module derived from generic_module
 * 
 * @author 
 */
class PDD_g_comp_module : public generic_module<PDD_g_comp_thread> {
public:
    
    /**
     * @brief constructor: do nothing but construct the superclass
     * 
     */
    PDD_g_comp_module(   int argc, 
                               char* argv[],
                               std::string module_prefix, 
                               int module_period, 
                               yarp::os::ResourceFinder rf ) : generic_module<PDD_g_comp_thread>(  argc, 
                                                                                            		argv, 
                                                                                            		module_prefix, 
                                                                                            		module_period,
                                                                                            		rf )
    {
    }
    
    /**
     * @brief overriden function to specify the custom params for the param helper
     * 
     * @return a vector of the custom params for the param helper
     */
    virtual std::vector< paramHelp::ParamProxyInterface* > custom_get_ph_parameters() 
    {
	// TODO: function skeleton
        std::vector<paramHelp::ParamProxyInterface *> custom_params;
        return custom_params;
    }
    
    
};

#endif
