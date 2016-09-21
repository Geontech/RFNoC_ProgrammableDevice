#ifndef RFNOC_PROGRAMMABLEDEVICE_I_IMPL_H
#define RFNOC_PROGRAMMABLEDEVICE_I_IMPL_H

#include "RFNoC_ProgrammableDevice_prog_base.h"

#include <uhd/image_loader.hpp>

typedef RFNoC_ProgrammableDevice_prog_base<HW_LOAD::default_hw_load_request_struct, HW_LOAD::default_hw_load_status_struct> RFNoC_ProgrammableDevice_prog_base_type;


class RFNoC_ProgrammableDevice_i;

class RFNoC_ProgrammableDevice_i : public RFNoC_ProgrammableDevice_prog_base_type
{
    ENABLE_LOGGING
    public:
        RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl);
        RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev);
        RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities);
        RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev);
        ~RFNoC_ProgrammableDevice_i();
        int serviceFunction();
        void initialize() throw (CF::LifeCycle::InitializeError, CORBA::SystemException);
        void releaseObject() throw (CF::LifeCycle::ReleaseError, CORBA::SystemException);

    protected:
        Device_impl* generatePersona(int argc, char* argv[], ConstructorPtr fnptr, const char* libName);
        bool loadHardware(HwLoadStatusStruct& requestStatus);
        void unloadHardware(const HwLoadStatusStruct& requestStatus);

    private:
        uhd::image_loader::image_loader_args_t image_loader_args;
};

#endif // RFNOC_PROGRAMMABLEDEVICE_I_IMPL_H
