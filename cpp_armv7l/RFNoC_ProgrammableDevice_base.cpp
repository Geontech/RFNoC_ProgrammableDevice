#include "RFNoC_ProgrammableDevice_base.h"

/*******************************************************************************************

    AUTO-GENERATED CODE. DO NOT MODIFY

    The following class functions are for the base class for the device class. To
    customize any of these functions, do not modify them here. Instead, overload them
    on the child class

******************************************************************************************/

RFNoC_ProgrammableDevice_base::RFNoC_ProgrammableDevice_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl) :
    ExecutableDevice_impl(devMgr_ior, id, lbl, sftwrPrfl),
    AggregateDevice_impl(),
    ThreadedComponent()
{
    construct();
}

RFNoC_ProgrammableDevice_base::RFNoC_ProgrammableDevice_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev) :
    ExecutableDevice_impl(devMgr_ior, id, lbl, sftwrPrfl, compDev),
    AggregateDevice_impl(),
    ThreadedComponent()
{
    construct();
}

RFNoC_ProgrammableDevice_base::RFNoC_ProgrammableDevice_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities) :
    ExecutableDevice_impl(devMgr_ior, id, lbl, sftwrPrfl, capacities),
    AggregateDevice_impl(),
    ThreadedComponent()
{
    construct();
}

RFNoC_ProgrammableDevice_base::RFNoC_ProgrammableDevice_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev) :
    ExecutableDevice_impl(devMgr_ior, id, lbl, sftwrPrfl, capacities, compDev),
    AggregateDevice_impl(),
    ThreadedComponent()
{
    construct();
}

RFNoC_ProgrammableDevice_base::~RFNoC_ProgrammableDevice_base()
{
    delete DigitalTuner_in;
    DigitalTuner_in = 0;
    delete RFInfo_in;
    RFInfo_in = 0;
    delete dataShort_out;
    dataShort_out = 0;
}

void RFNoC_ProgrammableDevice_base::construct()
{
    loadProperties();

    DigitalTuner_in = new FRONTEND_DigitalTuner_In_i("DigitalTuner_in", this);
    addPort("DigitalTuner_in", DigitalTuner_in);
    RFInfo_in = new FRONTEND_RFInfo_In_i("RFInfo_in", this);
    addPort("RFInfo_in", RFInfo_in);
    dataShort_out = new bulkio::OutShortPort("dataShort_out");
    addPort("dataShort_out", dataShort_out);

    this->addPropertyListener(connectionTable, this, &RFNoC_ProgrammableDevice_base::connectionTableChanged);

}

/*******************************************************************************************
    Framework-level functions
    These functions are generally called by the framework to perform housekeeping.
*******************************************************************************************/
void RFNoC_ProgrammableDevice_base::start() throw (CORBA::SystemException, CF::Resource::StartError)
{
    ExecutableDevice_impl::start();
    ThreadedComponent::startThread();
}

void RFNoC_ProgrammableDevice_base::stop() throw (CORBA::SystemException, CF::Resource::StopError)
{
    ExecutableDevice_impl::stop();
    if (!ThreadedComponent::stopThread()) {
        throw CF::Resource::StopError(CF::CF_NOTSET, "Processing thread did not die");
    }
}

void RFNoC_ProgrammableDevice_base::releaseObject() throw (CORBA::SystemException, CF::LifeCycle::ReleaseError)
{
    // This function clears the device running condition so main shuts down everything
    try {
        stop();
    } catch (CF::Resource::StopError& ex) {
        // TODO - this should probably be logged instead of ignored
    }

    ExecutableDevice_impl::releaseObject();
}

void RFNoC_ProgrammableDevice_base::connectionTableChanged(const std::vector<connection_descriptor_struct>* oldValue, const std::vector<connection_descriptor_struct>* newValue)
{
    dataShort_out->updateConnectionFilter(*newValue);
}

void RFNoC_ProgrammableDevice_base::loadProperties()
{
    addProperty(device_kind,
                "FRONTEND::TUNER",
                "DCE:cdc5ee18-7ceb-4ae6-bf4c-31f983179b4d",
                "device_kind",
                "readonly",
                "",
                "eq",
                "allocation");

    addProperty(device_model,
                "USRP_UHD_RFNoC",
                "DCE:0f99b2e4-9903-4631-9846-ff349d18ecfb",
                "device_model",
                "readonly",
                "",
                "eq",
                "allocation");

    addProperty(processor_name,
                "RF-NoC",
                "DCE:9B445600-6C7F-11d4-A226-0050DA314CD6",
                "processor_name",
                "readonly",
                "",
                "eq",
                "allocation");

    addProperty(os_name,
                "Linux",
                "DCE:80BF17F0-6C7F-11d4-A226-0050DA314CD6",
                "os_name",
                "readonly",
                "",
                "eq",
                "allocation");

    addProperty(os_version,
                "3.14.2-xilinx",
                "DCE:0f3a9a37-a342-43d8-9b7f-78dc6da74192",
                "os_version",
                "readonly",
                "",
                "eq",
                "allocation");

    addProperty(frontend_listener_allocation,
                frontend_listener_allocation_struct(),
                "FRONTEND::listener_allocation",
                "frontend_listener_allocation",
                "readwrite",
                "",
                "external",
                "allocation");

    addProperty(frontend_tuner_allocation,
                frontend_tuner_allocation_struct(),
                "FRONTEND::tuner_allocation",
                "frontend_tuner_allocation",
                "readwrite",
                "",
                "external",
                "allocation");

    addProperty(hw_load_requests,
                "hw_load_requests",
                "",
                "readwrite",
                "",
                "external",
                "allocation");

    addProperty(hw_load_statuses,
                "hw_load_statuses",
                "",
                "readwrite",
                "",
                "external",
                "property");

    addProperty(frontend_tuner_status,
                "FRONTEND::tuner_status",
                "frontend_tuner_status",
                "readonly",
                "",
                "external",
                "property");

    addProperty(connectionTable,
                "connectionTable",
                "",
                "readonly",
                "",
                "external",
                "property");

}


