#ifndef RFNOC_PROGRAMMABLEDEVICE_I_IMPL_H
#define RFNOC_PROGRAMMABLEDEVICE_I_IMPL_H

#include "RFNoC_ProgrammableDevice_prog_base.h"

// Change the template to use the property versions of these structures
// This allows users to see the available loads and perform an allocation
typedef RFNoC_ProgrammableDevice_prog_base<hw_load_request_struct_struct, hw_load_statuses_struct_struct> RFNoC_ProgrammableDevice_prog_base_type;

#include <uhd/usrp/multi_usrp.hpp>

#include "HwLoadStatus.h"

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

        void setHwLoadStatus(const hw_load_status_object &hwLoadStatus);

    protected:
        Device_impl* generatePersona(int argc, char* argv[], ConstructorPtr fnptr, const char* libName);
        bool loadHardware(HwLoadStatusStruct& requestStatus);
        void unloadHardware(const HwLoadStatusStruct& requestStatus);

        virtual bool hwLoadRequestIsValid(const HwLoadRequestStruct& hwLoadRequestStruct);

    private:
        const std::string HARDWARE_ID;
        uhd::usrp::multi_usrp::sptr usrp;
};

#endif // RFNOC_PROGRAMMABLEDEVICE_I_IMPL_H
