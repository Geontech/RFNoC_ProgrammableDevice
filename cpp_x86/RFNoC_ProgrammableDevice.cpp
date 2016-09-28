/**************************************************************************

    This is the device code. This file contains the child class where
    custom functionality can be added to the device. Custom
    functionality to the base class can be extended here. Access to
    the ports can also be done from this class

**************************************************************************/

#include "RFNoC_ProgrammableDevice.h"

#include <boost/filesystem.hpp>

#include "image_loader.hpp"

PREPARE_LOGGING(RFNoC_ProgrammableDevice_i)

RFNoC_ProgrammableDevice_i::RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl) :
    RFNoC_ProgrammableDevice_prog_base_type(devMgr_ior, id, lbl, sftwrPrfl),
    HARDWARE_ID("X3X0")
{
}

RFNoC_ProgrammableDevice_i::RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev) :
    RFNoC_ProgrammableDevice_prog_base_type(devMgr_ior, id, lbl, sftwrPrfl, compDev),
    HARDWARE_ID("X3X0")
{
}

RFNoC_ProgrammableDevice_i::RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities) :
    RFNoC_ProgrammableDevice_prog_base_type(devMgr_ior, id, lbl, sftwrPrfl, capacities),
    HARDWARE_ID("X3X0")
{
}

RFNoC_ProgrammableDevice_i::RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev) :
    RFNoC_ProgrammableDevice_prog_base_type(devMgr_ior, id, lbl, sftwrPrfl, capacities, compDev),
    HARDWARE_ID("X3X0")
{
}

RFNoC_ProgrammableDevice_i::~RFNoC_ProgrammableDevice_i()
{
}

void RFNoC_ProgrammableDevice_i::initialize() throw (CF::LifeCycle::InitializeError, CORBA::SystemException) 
{
    setHwLoadRequestsPtr(&hw_load_requests);
    setHwLoadStatusesPtr(&hw_load_statuses);
}

int RFNoC_ProgrammableDevice_i::serviceFunction()
{
    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "serviceFunction() example log message");
    
    return NOOP;
}

void RFNoC_ProgrammableDevice_i::setHwLoadStatus(const hw_load_status_object &hwLoadStatus)
{
    LOG_INFO(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    hw_load_statuses_struct_struct hwLoadStatusStruct;
    hwLoadStatusStruct.hardware_id = hwLoadStatus.hardware_id;
    hwLoadStatusStruct.load_filepath = hwLoadStatus.load_filepath;
    hwLoadStatusStruct.request_id = hwLoadStatus.request_id;
    hwLoadStatusStruct.requester_id = hwLoadStatus.requester_id;
    hwLoadStatusStruct.state = hwLoadStatus.state;

    this->hw_load_statuses.push_back(hwLoadStatusStruct);
}

Device_impl* RFNoC_ProgrammableDevice_i::generatePersona(int argc, char* argv[], ConstructorPtr personaEntryPoint, const char* libName)
{
    // Generate the Persona Device
    Device_impl *persona = personaEntryPoint(argc, argv, this, boost::bind(&RFNoC_ProgrammableDevice_i::setHwLoadStatus, this, _1));

    // Something went wrong
    if (not persona) {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Failed to generate Persona Device. Unable to instantiate Persona Device from library: " << libName);
        return NULL;
    }

    return persona;
}

bool RFNoC_ProgrammableDevice_i::loadHardware(HwLoadStatusStruct& requestStatus) 
{
    // The hardware may be physically loaded at this point
    LOG_INFO(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    uhd::image_loader::image_loader_args_t loader_args;
    loader_args.firmware_path = "";
    loader_args.fpga_path = requestStatus.load_filepath;
    loader_args.load_firmware = false;
    loader_args.load_fpga = true;

    if (not uhd::image_loader::load(loader_args)) {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Failed to load hardware.");
        return false;
    }

    return true;
}

void RFNoC_ProgrammableDevice_i::unloadHardware(const HwLoadStatusStruct& requestStatus) 
{
    // The hardware may be physically unloaded at this point
    LOG_INFO(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    uhd::image_loader::image_loader_args_t loader_args;
    loader_args.firmware_path = "";
    loader_args.fpga_path = "/usr/share/uhd/images/usrp_x300_fpga_HGS.bit";
    loader_args.load_firmware = false;
    loader_args.load_fpga = true;

    if (not uhd::image_loader::load(loader_args)) {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Failed to unload hardware.");
    }
}

bool RFNoC_ProgrammableDevice_i::hwLoadRequestIsValid(const HwLoadRequestStruct& hwLoadRequestStruct)
{
    if (hwLoadRequestStruct.hardware_id != this->HARDWARE_ID) {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to validate hardware load request. Mismatched hardware IDs.");
        return false;
    }

    if (not boost::filesystem::exists(hwLoadRequestStruct.load_filepath)) {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to validate hardware load request. Load file path is invalid.");
        return false;
    }

    return true;
}
