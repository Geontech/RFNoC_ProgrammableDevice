#ifndef RFNOC_PROGRAMMABLEDEVICE_BASE_IMPL_BASE_H
#define RFNOC_PROGRAMMABLEDEVICE_BASE_IMPL_BASE_H

#include <boost/thread.hpp>
#include <ossie/ExecutableDevice_impl.h>
#include <CF/AggregateDevices.h>
#include <ossie/AggregateDevice_impl.h>
#include <ossie/ThreadedComponent.h>

#include <bulkio/bulkio.h>
#include "struct_props.h"
#include "port_impl.h"

class RFNoC_ProgrammableDevice_base : public ExecutableDevice_impl, public virtual POA_CF::AggregateExecutableDevice, public AggregateDevice_impl, protected ThreadedComponent
{
    friend class FRONTEND_DigitalTuner_In_i;
    friend class FRONTEND_RFInfo_In_i;

    public:
        RFNoC_ProgrammableDevice_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl);
        RFNoC_ProgrammableDevice_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev);
        RFNoC_ProgrammableDevice_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities);
        RFNoC_ProgrammableDevice_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev);
        ~RFNoC_ProgrammableDevice_base();

        void start() throw (CF::Resource::StartError, CORBA::SystemException);

        void stop() throw (CF::Resource::StopError, CORBA::SystemException);

        void releaseObject() throw (CF::LifeCycle::ReleaseError, CORBA::SystemException);

        void loadProperties();

    protected:
        void connectionTableChanged(const std::vector<connection_descriptor_struct>* oldValue, const std::vector<connection_descriptor_struct>* newValue);

        // Member variables exposed as properties
        /// Property: device_kind
        std::string device_kind;
        /// Property: device_model
        std::string device_model;
        /// Property: processor_name
        std::string processor_name;
        /// Property: os_name
        std::string os_name;
        /// Property: os_version
        std::string os_version;
        /// Property: frontend_listener_allocation
        frontend_listener_allocation_struct frontend_listener_allocation;
        /// Property: frontend_tuner_allocation
        frontend_tuner_allocation_struct frontend_tuner_allocation;
        /// Property: target_device
        target_device_struct target_device;
        /// Property: hw_load_requests
        std::vector<hw_load_request_struct_struct> hw_load_requests;
        /// Property: hw_load_statuses
        std::vector<hw_load_statuses_struct_struct> hw_load_statuses;
        /// Property: frontend_tuner_status
        std::vector<frontend_tuner_status_struct_struct> frontend_tuner_status;
        /// Property: connectionTable
        std::vector<connection_descriptor_struct> connectionTable;

        // Ports
        /// Port: DigitalTuner_in
        FRONTEND_DigitalTuner_In_i *DigitalTuner_in;
        /// Port: RFInfo_in
        FRONTEND_RFInfo_In_i *RFInfo_in;
        /// Port: dataShort_out
        bulkio::OutShortPort *dataShort_out;

    private:
        void construct();
};
#endif // RFNOC_PROGRAMMABLEDEVICE_BASE_IMPL_BASE_H
