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
    HARDWARE_ID("E310"),
    IDLE_BITFILE_PATH("/usr/share/uhd/images/usrp_e3xx_fpga_idle.bit")
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

RFNoC_ProgrammableDevice_i::RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev) :
    RFNoC_ProgrammableDevice_prog_base_type(devMgr_ior, id, lbl, sftwrPrfl, compDev),
    HARDWARE_ID("E310"),
    IDLE_BITFILE_PATH("/usr/share/uhd/images/usrp_e3xx_fpga_idle.bit")
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

RFNoC_ProgrammableDevice_i::RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities) :
    RFNoC_ProgrammableDevice_prog_base_type(devMgr_ior, id, lbl, sftwrPrfl, capacities),
    HARDWARE_ID("E310"),
    IDLE_BITFILE_PATH("/usr/share/uhd/images/usrp_e3xx_fpga_idle.bit")
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

RFNoC_ProgrammableDevice_i::RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev) :
    RFNoC_ProgrammableDevice_prog_base_type(devMgr_ior, id, lbl, sftwrPrfl, capacities, compDev),
    HARDWARE_ID("E310"),
    IDLE_BITFILE_PATH("/usr/share/uhd/images/usrp_e3xx_fpga_idle.bit")
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

RFNoC_ProgrammableDevice_i::~RFNoC_ProgrammableDevice_i()
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

void RFNoC_ProgrammableDevice_i::initialize() throw (CF::LifeCycle::InitializeError, CORBA::SystemException) 
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Set the load requests and statuses pointers to the properties
    setHwLoadRequestsPtr(&hw_load_requests);
    setHwLoadStatusesPtr(&hw_load_statuses);

    // Set the usrp address
    this->usrpAddress["name"] = this->target_device.name;
    this->usrpAddress["no_reload_fpga"] = true;

    if (not this->target_device.serial.empty()) {
        this->usrpAddress["serial"] = this->target_device.serial;
    }

    if (not this->target_device.type.empty()) {
        this->usrpAddress["type"] = this->target_device.type;
    }

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Trying target_device: " << this->usrpAddress.to_string());

    // Reset device streaming state
    //this->usrp->clear();

    // Initialize the radios
    //initializeRadios();

    // Register the property change listener
    this->addPropertyListener(this->target_device, this, &RFNoC_ProgrammableDevice_i::target_deviceChanged);

    // Register the frontend callbacks
    this->setAllocationImpl(this->frontend_listener_allocation, this, &RFNoC_ProgrammableDevice_i::frontend_listener_allocation_alloc, &RFNoC_ProgrammableDevice_i::frontend_listener_allocation_dealloc);
    this->setAllocationImpl(this->frontend_tuner_allocation, this, &RFNoC_ProgrammableDevice_i::frontend_tuner_allocation_alloc, &RFNoC_ProgrammableDevice_i::frontend_tuner_allocation_dealloc);
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
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Generate the Persona Device
    Device_impl *persona = personaEntryPoint(argc, argv, this, boost::bind(&RFNoC_ProgrammableDevice_i::setHwLoadStatus, this, _1), this->usrpAddress);

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

    // Load the requested bitfile
    uhd::image_loader::image_loader_args_t image_loader_args;

    image_loader_args.firmware_path = "";
    image_loader_args.fpga_path = requestStatus.load_filepath;
    image_loader_args.load_firmware = false;
    image_loader_args.load_fpga = true;

    uhd::image_loader::load(image_loader_args);

    // Allow some time for setup
    boost::this_thread::sleep(boost::posix_time::seconds(1.0));

    // Attempt to get a reference to the specified device
    try {
        this->usrp = uhd::device3::make(this->usrpAddress);
    } catch(uhd::key_error &e) {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Unable to find a suitable USRP Device 3.");
        LOG_ERROR(RFNoC_ProgrammableDevice_i, e.what());
        return false;
    } catch(...) {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "An error occurred attempting to get a reference to the USRP device.");
        return false;
    }

    // Attempt to get the radios, DDCs, and DUCs
    initializeRadioChain();

    return true;
}

void RFNoC_ProgrammableDevice_i::unloadHardware(const HwLoadStatusStruct& requestStatus) 
{
    // The hardware may be physically unloaded at this point
    LOG_INFO(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Load the idle bitfile
    uhd::image_loader::image_loader_args_t image_loader_args;

    image_loader_args.firmware_path = "";
    image_loader_args.fpga_path = this->IDLE_BITFILE_PATH;
    image_loader_args.load_firmware = false;
    image_loader_args.load_fpga = true;

    uhd::image_loader::load(image_loader_args);

    // Clear the USRP pointer
    this->usrp.reset();

    // Clear the radios, DDCs, and DUCs
    this->radios.clear();
    this->ddcs.clear();
    this->ducs.clear();
}

bool RFNoC_ProgrammableDevice_i::hwLoadRequestIsValid(const HwLoadRequestStruct& hwLoadRequestStruct)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    if (hwLoadRequestStruct.hardware_id != this->HARDWARE_ID) {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to validate hardware load request. Mismatched hardware IDs.");
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Expected: " << this->HARDWARE_ID << ", Received: " << hwLoadRequestStruct.hardware_id);
        return false;
    }

    if (not boost::filesystem::exists(hwLoadRequestStruct.load_filepath)) {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to validate hardware load request. Load file path is invalid.");
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Invalid file path: " << hwLoadRequestStruct.load_filepath);
        return false;
    }

    return true;
}

void RFNoC_ProgrammableDevice_i::initializeRadioChain()
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Variables for the number of channels
    size_t num_rx_channels = 0;
    size_t num_tx_channels = 0;

    // Grab the radio blocks
    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Searching for radio blocks");
    std::vector<uhd::rfnoc::block_id_t> radio_block_ids = this->usrp->find_blocks("Radio");

    for (size_t i = 0; i < radio_block_ids.size(); ++i) {
        uhd::rfnoc::block_id_t blockId = radio_block_ids[i];

        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Found radio block with ID: " << blockId.to_string());

        uhd::rfnoc::radio_ctrl::sptr radio = this->usrp->get_block_ctrl<uhd::rfnoc::radio_ctrl>(blockId);

        num_rx_channels += radio->get_output_ports().size();
        num_tx_channels += radio->get_input_ports().size();

        this->radios.push_back(radio);
    }

    // Grab the DDC blocks
    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Searching for DDC blocks");
    std::vector<uhd::rfnoc::block_id_t> ddc_block_ids = this->usrp->find_blocks("DDC");

    for (size_t i = 0; i < ddc_block_ids.size(); ++i) {
        uhd::rfnoc::block_id_t blockId = ddc_block_ids[i];

        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Found DDC block with ID: " << blockId.to_string());

        uhd::rfnoc::ddc_block_ctrl::sptr ddc = this->usrp->get_block_ctrl<uhd::rfnoc::ddc_block_ctrl>(blockId);

        this->ddcs.push_back(ddc);
    }

    // Grab the DUC blocks
    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Searching for DUC blocks");
    std::vector<uhd::rfnoc::block_id_t> duc_block_ids = this->usrp->find_blocks("DUC");

    for (size_t i = 0; i < duc_block_ids.size(); ++i) {
        uhd::rfnoc::block_id_t blockId = duc_block_ids[i];

        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Found DUC block with ID: " << blockId.to_string());

        uhd::rfnoc::duc_block_ctrl::sptr duc = this->usrp->get_block_ctrl<uhd::rfnoc::duc_block_ctrl>(blockId);

        this->ducs.push_back(duc);
    }

    setNumChannels(num_rx_channels + num_tx_channels);

    size_t i;

    for (i = 0; i < num_rx_channels; ++i) {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "Adding an RX channel");
        this->rxStatuses.push_back(&this->frontend_tuner_status[i]);
    }

    for (; i < this->frontend_tuner_status.size(); ++i) {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "Adding a TX channel");
        this->frontend_tuner_status[i].tuner_type = "TX";

        this->txStatuses.push_back(&this->frontend_tuner_status[i]);
    }

    LOG_INFO(RFNoC_ProgrammableDevice_i, "There are " << this->rxStatuses.size() << " RX channels");
    LOG_INFO(RFNoC_ProgrammableDevice_i, "There are " << this->txStatuses.size() << " TX channels");

    /*for (size_t i = 0; i < this->rxStatuses.size(); ++i) {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "Gathering info for RX Channel " << i);
        frontend_tuner_status_struct_struct &fts = *this->rxStatuses[i];

        double bw = this->usrp->get_rx_bandwidth(i);

        fts.bandwidth = bw;

        uhd::freq_range_t bwRange = this->usrp->get_rx_bandwidth_range(i);

        std::stringstream ss;

        for (size_t j = 0; j < bwRange.size(); ++j) {
            if (j != 0) {
                ss << ",";
            }

            ss << std::fixed << bwRange[j].start() << "-" << bwRange.stop();
        }

        fts.available_bandwidth = ss.str();

        ss.str(std::string());

        double cf = this->usrp->get_rx_freq(i);

        fts.center_frequency = cf;

        uhd::freq_range_t cfRange = this->usrp->get_rx_freq_range(i);

        for (size_t j = 0; j < cfRange.size(); ++j) {
            if (j != 0) {
                ss << ",";
            }

            ss << std::fixed << cfRange[j].start() << "-" << cfRange[j].stop();
        }

        fts.available_frequency = ss.str();

        ss.str(std::string());

        double sr = this->usrp->get_rx_rate(i);

        fts.sample_rate = sr;

        uhd::meta_range_t srRange = this->usrp->get_rx_rates(i);

        for (size_t j = 0; j < srRange.size(); ++j) {
            if (j != 0) {
                ss << ",";
            }

            ss << std::fixed << srRange[j].start() << "-" << srRange[j].stop();
        }

        fts.available_sample_rate = ss.str();
    }

    for (size_t i = 0; i < this->txStatuses.size(); ++i) {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "Gathering info for TX Channel " << i);
        frontend_tuner_status_struct_struct &fts = *this->txStatuses[i];

        double bw = this->usrp->get_tx_bandwidth(i);

        fts.bandwidth = bw;

        uhd::freq_range_t bwRange = this->usrp->get_tx_bandwidth_range(i);

        std::stringstream ss;

        for (size_t j = 0; j < bwRange.size(); ++j) {
            if (j != 0) {
                ss << ",";
            }

            ss << std::fixed << bwRange[j].start() << "-" << bwRange.stop();
        }

        fts.available_bandwidth = ss.str();

        ss.str(std::string());

        double cf = this->usrp->get_tx_freq(i);

        fts.center_frequency = cf;

        uhd::freq_range_t cfRange = this->usrp->get_tx_freq_range(i);

        for (size_t j = 0; j < cfRange.size(); ++j) {
            if (j != 0) {
                ss << ",";
            }

            ss << std::fixed << cfRange[j].start() << "-" << cfRange[j].stop();
        }

        fts.available_frequency = ss.str();

        ss.str(std::string());

        double sr = this->usrp->get_tx_rate(i);

        fts.sample_rate = sr;

        uhd::meta_range_t srRange = this->usrp->get_tx_rates(i);

        for (size_t j = 0; j < srRange.size(); ++j) {
            if (j != 0) {
                ss << ",";
            }

            ss << std::fixed << srRange[j].start() << "-" << srRange[j].stop();
        }

        fts.available_sample_rate = ss.str();
    }*/
}

void RFNoC_ProgrammableDevice_i::target_deviceChanged(const target_device_struct &oldValue, const target_device_struct &newValue)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to set the target_device while running. Must be set in DCD file.");

    this->target_device = oldValue;
}

CF::Device::UsageType RFNoC_ProgrammableDevice_i::updateUsageState() {
    //LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);
    size_t tunerAllocated = 0;
    for (size_t tuner_id = 0; tuner_id < tuner_allocation_ids.size(); tuner_id++) {
        if (!tuner_allocation_ids[tuner_id].control_allocation_id.empty())
            tunerAllocated++;
    }
    // If no tuners are allocated, device is idle
    if (tunerAllocated == 0)
        return CF::Device::IDLE;
    // If all tuners are allocated, device is busy
    if (tunerAllocated == tuner_allocation_ids.size())
        return CF::Device::BUSY;
    // Else, device is active
    return CF::Device::ACTIVE;
}

/*************************************************************
Functions supporting tuning allocation
*************************************************************/
void RFNoC_ProgrammableDevice_i::deviceEnable(frontend_tuner_status_struct_struct &fts,
                                        size_t tuner_id)
{
    /************************************************************
    modify fts, which corresponds to this->frontend_tuner_status[tuner_id]
    Make sure to set the 'enabled' member of fts to indicate that tuner as
    enabled
    ************************************************************/
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    fts.enabled = true;
    return;
}

void RFNoC_ProgrammableDevice_i::deviceDisable(frontend_tuner_status_struct_struct &fts,
                                        size_t tuner_id)
{
    /************************************************************
    modify fts, which corresponds to this->frontend_tuner_status[tuner_id]
    Make sure to reset the 'enabled' member of fts to indicate that tuner as
    disabled
    ************************************************************/
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    fts.enabled = false;
    return;
}

bool RFNoC_ProgrammableDevice_i::deviceSetTuning(
        const frontend_tuner_allocation_struct &request,
        frontend_tuner_status_struct_struct &fts, size_t tuner_id)
{
    /************************************************************
    modify fts, which corresponds to this->frontend_tuner_status[tuner_id]
    return true if the tuning succeeded, and false if it failed
    ************************************************************/
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    return true;
}

bool RFNoC_ProgrammableDevice_i::deviceDeleteTuning(
        frontend_tuner_status_struct_struct &fts, size_t tuner_id)
{
    /************************************************************
    modify fts, which corresponds to this->frontend_tuner_status[tuner_id]
    return true if the tune deletion succeeded, and false if it failed
    ************************************************************/
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    return true;
}

std::string RFNoC_ProgrammableDevice_i::createAllocationIdCsv(size_t tuner_id){
    //LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);
    std::string alloc_id_csv = "";
    // ensure control allocation_id is first in list
    if (!tuner_allocation_ids[tuner_id].control_allocation_id.empty())
        alloc_id_csv = tuner_allocation_ids[tuner_id].control_allocation_id + ",";
    std::vector<std::string>::iterator it = tuner_allocation_ids[tuner_id].listener_allocation_ids.begin();
    for(; it != tuner_allocation_ids[tuner_id].listener_allocation_ids.end(); it++)
        alloc_id_csv += *it + ",";
    if(!alloc_id_csv.empty())
        alloc_id_csv.erase(alloc_id_csv.size()-1);
    return alloc_id_csv;
}

std::string RFNoC_ProgrammableDevice_i::getControlAllocationId(size_t tuner_id){
    return tuner_allocation_ids[tuner_id].control_allocation_id;
}

std::vector<std::string> RFNoC_ProgrammableDevice_i::getListenerAllocationIds(size_t tuner_id){
    return tuner_allocation_ids[tuner_id].listener_allocation_ids;
}

bool RFNoC_ProgrammableDevice_i::enableTuner(size_t tuner_id, bool enable) {
    //LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);

    bool prev_enabled = frontend_tuner_status[tuner_id].enabled;

    // If going from disabled to enabled
    if (!prev_enabled && enable) {
        deviceEnable(frontend_tuner_status[tuner_id], tuner_id);
    }

    // If going from enabled to disabled
    if (prev_enabled && !enable) {

        deviceDisable(frontend_tuner_status[tuner_id], tuner_id);
    }

    return true;
}

bool RFNoC_ProgrammableDevice_i::listenerRequestValidation(frontend_tuner_allocation_struct &request, size_t tuner_id){
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);

    // ensure requested values are non-negative
    if(frontend::floatingPointCompare(request.center_frequency,0)<0 || frontend::floatingPointCompare(request.bandwidth,0)<0 || frontend::floatingPointCompare(request.sample_rate,0)<0 || frontend::floatingPointCompare(request.bandwidth_tolerance,0)<0 || frontend::floatingPointCompare(request.sample_rate_tolerance,0)<0)
        return false;

    // ensure lower end of requested band fits
    //if((request.center_frequency - (request.bandwidth*0.5)) < (frontend_tuner_status[tuner_id].center_frequency - (frontend_tuner_status[tuner_id].bandwidth*0.5))){
    if( frontend::floatingPointCompare((request.center_frequency-(request.bandwidth*0.5)),(frontend_tuner_status[tuner_id].center_frequency-(frontend_tuner_status[tuner_id].bandwidth*0.5))) < 0 ){
        LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__ << " FAILED LOWER END TEST");
        return false;
    }

    // ensure upper end of requested band fits
    //if((request.center_frequency + (request.bandwidth*0.5)) > (frontend_tuner_status[tuner_id].center_frequency + (frontend_tuner_status[tuner_id].bandwidth*0.5))){
    if( frontend::floatingPointCompare((request.center_frequency + (request.bandwidth*0.5)),(frontend_tuner_status[tuner_id].center_frequency + (frontend_tuner_status[tuner_id].bandwidth*0.5))) > 0 ){
        LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__ << " FAILED UPPER END TEST");
        return false;
    }

    // ensure tuner bandwidth meets requested tolerance
    //if(request.bandwidth > frontend_tuner_status[tuner_id].bandwidth)
    if( frontend::floatingPointCompare(request.bandwidth,frontend_tuner_status[tuner_id].bandwidth) > 0 )
        return false;

    //if(request.bandwidth != 0 && (request.bandwidth+(request.bandwidth*request.bandwidth_tolerance/100)) < frontend_tuner_status[tuner_id].bandwidth)
    if( frontend::floatingPointCompare(request.bandwidth,0)!=0 && frontend::floatingPointCompare((request.bandwidth+(request.bandwidth*request.bandwidth_tolerance/100)),frontend_tuner_status[tuner_id].bandwidth) < 0 )
        return false;

    // ensure tuner sample rate meets requested tolerance
    //if(request.sample_rate > frontend_tuner_status[tuner_id].sample_rate)
    if( frontend::floatingPointCompare(request.sample_rate,frontend_tuner_status[tuner_id].sample_rate) > 0 )
        return false;

    //if(request.sample_rate != 0 && (request.sample_rate+(request.sample_rate*request.sample_rate_tolerance/100)) < frontend_tuner_status[tuner_id].sample_rate)
    if(frontend::floatingPointCompare(request.sample_rate,0)!=0 && frontend::floatingPointCompare((request.sample_rate+(request.sample_rate*request.sample_rate_tolerance/100)),frontend_tuner_status[tuner_id].sample_rate) < 0 )
        return false;

    return true;
};

////////////////////////////
//        MAPPING         //
////////////////////////////

long RFNoC_ProgrammableDevice_i::getTunerMapping(std::string allocation_id) {
    //LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);
    long NO_VALID_TUNER = -1;

    string_number_mapping::iterator iter = allocation_id_to_tuner_id.find(allocation_id);
    if (iter != allocation_id_to_tuner_id.end())
        return iter->second;

    return NO_VALID_TUNER;
}

bool RFNoC_ProgrammableDevice_i::removeTunerMapping(size_t tuner_id, std::string allocation_id) {
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);
    removeListener(allocation_id);
    std::vector<std::string>::iterator it = tuner_allocation_ids[tuner_id].listener_allocation_ids.begin();
    while(it != tuner_allocation_ids[tuner_id].listener_allocation_ids.end()){
        if(*it == allocation_id){
            tuner_allocation_ids[tuner_id].listener_allocation_ids.erase(it);
        } else {
            ++it;
        }
    }
    exclusive_lock lock(allocation_id_mapping_lock);
    if(allocation_id_to_tuner_id.erase(allocation_id) > 0)
        return true;
    return false;
}

bool RFNoC_ProgrammableDevice_i::removeTunerMapping(size_t tuner_id) {
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);
    deviceDeleteTuning(frontend_tuner_status[tuner_id], tuner_id);
    removeAllocationIdRouting(tuner_id);

    long cnt = 0;
    exclusive_lock lock(allocation_id_mapping_lock);
    string_number_mapping::iterator it = allocation_id_to_tuner_id.begin();
    while(it != allocation_id_to_tuner_id.end()){
        if(it->second == tuner_id){
            std::string allocation_id = it->first;
            removeListener(allocation_id);
            allocation_id_to_tuner_id.erase(it++);
            cnt++;
        } else {
            ++it;
        }
    }
    /*
    for(std::vector<std::string>::iterator it = tuner_allocation_ids[tuner_id].listener_allocation_ids.begin(); it != tuner_allocation_ids[tuner_id].listener_allocation_ids.end();it++){
        removeListener(*it);
        allocation_id_to_tuner_id.erase(*it);
        cnt++;
    }
    removeListener(tuner_allocation_ids[tuner_id].control_allocation_id);
    allocation_id_to_tuner_id.erase(tuner_allocation_ids[tuner_id].control_allocation_id);
    */
    tuner_allocation_ids[tuner_id].reset();
    return cnt > 0;
}

void RFNoC_ProgrammableDevice_i::assignListener(const std::string& listen_alloc_id, const std::string& alloc_id) {
};

void RFNoC_ProgrammableDevice_i::removeListener(const std::string& listen_alloc_id) {
};

void RFNoC_ProgrammableDevice_i::removeAllocationIdRouting(const size_t tuner_id) {
    std::string allocation_id = getControlAllocationId(tuner_id);
    std::vector<connection_descriptor_struct> old_table = this->connectionTable;
    std::vector<connection_descriptor_struct>::iterator itr = this->connectionTable.begin();
    while (itr != this->connectionTable.end()) {
        if (itr->connection_id == allocation_id) {
            itr = this->connectionTable.erase(itr);
            continue;
        }
        itr++;
    }
    for (std::map<std::string, std::string>::iterator listener=listeners.begin();listener!=listeners.end();listener++) {
        if (listener->second == allocation_id) {
            std::vector<connection_descriptor_struct>::iterator itr = this->connectionTable.begin();
            while (itr != this->connectionTable.end()) {
                if (itr->connection_id == listener->first) {
                    itr = this->connectionTable.erase(itr);
                    continue;
                }
                itr++;
            }
        }
    }
    this->connectionTableChanged(&old_table, &this->connectionTable);
}

/* This sets the number of entries in the frontend_tuner_status struct sequence property
 * as well as the tuner_allocation_ids vector. Call this function during initialization
 */
void RFNoC_ProgrammableDevice_i::setNumChannels(size_t num)
{
    this->setNumChannels(num, "RX_DIGITIZER");
}
/* This sets the number of entries in the frontend_tuner_status struct sequence property
 * as well as the tuner_allocation_ids vector. Call this function during initialization
 */

void RFNoC_ProgrammableDevice_i::setNumChannels(size_t num, std::string tuner_type)
{
    frontend_tuner_status.clear();
    frontend_tuner_status.resize(num);
    tuner_allocation_ids.clear();
    tuner_allocation_ids.resize(num);
    for (std::vector<frontend_tuner_status_struct_struct>::iterator iter=frontend_tuner_status.begin(); iter!=frontend_tuner_status.end(); iter++) {
        iter->enabled = false;
        iter->tuner_type = tuner_type;
    }
}

bool RFNoC_ProgrammableDevice_i::frontend_listener_allocation_alloc(const frontend_listener_allocation_struct &newAllocation)
{
    // Check validity of allocation_id's
    if (frontend_listener_allocation.existing_allocation_id.empty()){
        LOG_INFO(RFNoC_ProgrammableDevice_i,"allocateCapacity: MISSING EXISTING ALLOCATION ID");
        return false;
    }
    if (frontend_listener_allocation.listener_allocation_id.empty()){
        LOG_INFO(RFNoC_ProgrammableDevice_i,"allocateCapacity: MISSING LISTENER ALLOCATION ID");
        return false;
    }

    exclusive_lock lock(allocation_id_mapping_lock);

    // Check if listener allocation ID has already been used
    if(getTunerMapping(frontend_listener_allocation.listener_allocation_id) >= 0){
        LOG_INFO(RFNoC_ProgrammableDevice_i,"allocateCapacity: LISTENER ALLOCATION ID ALREADY IN USE: [" << frontend_listener_allocation.listener_allocation_id << "]");
        return false;
    }
    // Do not allocate if existing allocation ID does not exist
    long tuner_id = getTunerMapping(frontend_listener_allocation.existing_allocation_id);
    if (tuner_id < 0){
        LOG_DEBUG(RFNoC_ProgrammableDevice_i,"allocateCapacity: UNKNOWN CONTROL ALLOCATION ID: ["<< frontend_listener_allocation.existing_allocation_id <<"]");
        return false;
    }

    // listener allocations are not permitted for channelizers or TX
    if(frontend_tuner_status[tuner_id].tuner_type == "CHANNELIZER" || frontend_tuner_status[tuner_id].tuner_type == "TX"){
        std::ostringstream eout;
        eout<<"allocateCapacity: listener allocations are not permitted for " << std::string(frontend_tuner_status[tuner_id].tuner_type) << " tuner type";
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, eout.str());
        return false;
    }

    tuner_allocation_ids[tuner_id].listener_allocation_ids.push_back(frontend_listener_allocation.listener_allocation_id);
    allocation_id_to_tuner_id.insert(std::pair<std::string, size_t > (frontend_listener_allocation.listener_allocation_id, tuner_id));
    frontend_tuner_status[tuner_id].allocation_id_csv = createAllocationIdCsv(tuner_id);
    this->assignListener(frontend_listener_allocation.listener_allocation_id,frontend_listener_allocation.existing_allocation_id);

    return true;
}

void RFNoC_ProgrammableDevice_i::frontend_listener_allocation_dealloc(const frontend_listener_allocation_struct &newDeallocation)
{
    long tuner_id = getTunerMapping(frontend_listener_allocation.listener_allocation_id);
    if (tuner_id < 0){
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "ALLOCATION_ID NOT FOUND: [" << frontend_listener_allocation.listener_allocation_id <<"]");
        return;
    }

    // send EOS to listener connection only
    removeTunerMapping(tuner_id,frontend_listener_allocation.listener_allocation_id);
    frontend_tuner_status[tuner_id].allocation_id_csv = createAllocationIdCsv(tuner_id);
}

bool RFNoC_ProgrammableDevice_i::frontend_tuner_allocation_alloc(const frontend_tuner_allocation_struct &newAllocation)
{
    // Check allocation_id
    if (frontend_tuner_allocation.allocation_id.empty()) {
        LOG_INFO(RFNoC_ProgrammableDevice_i,"allocateCapacity: MISSING ALLOCATION_ID");
        return false;
    }
    // Check if allocation ID has already been used
    if(getTunerMapping(frontend_tuner_allocation.allocation_id) >= 0){
        LOG_INFO(RFNoC_ProgrammableDevice_i,"allocateCapacity: ALLOCATION_ID ALREADY IN USE: [" << frontend_tuner_allocation.allocation_id << "]");
        return false;
    }

    // Check if available tuner
    exclusive_lock lock(allocation_id_mapping_lock);

    // Next, try to allocate a new tuner
    for (size_t tuner_id = 0; tuner_id < tuner_allocation_ids.size(); tuner_id++) {
        if(frontend_tuner_status[tuner_id].tuner_type != frontend_tuner_allocation.tuner_type) {
            LOG_DEBUG(RFNoC_ProgrammableDevice_i,
              "allocateCapacity: Requested tuner type '"<<frontend_tuner_allocation.tuner_type <<"' does not match tuner[" << tuner_id << "].tuner_type ("<<frontend_tuner_status[tuner_id].tuner_type<<")");
            continue;
        }

        if(!frontend_tuner_allocation.group_id.empty() && frontend_tuner_allocation.group_id != frontend_tuner_status[tuner_id].group_id ){
            LOG_DEBUG(RFNoC_ProgrammableDevice_i,
              "allocateCapacity: Requested group_id '"<<frontend_tuner_allocation.group_id <<"' does not match tuner[" << tuner_id << "].group_id ("<<frontend_tuner_status[tuner_id].group_id<<")");
            continue;
        }

        // special case because allocation is specifying the input stream, which determines the rf_flow_id, etc.
        if(!frontend_tuner_allocation.rf_flow_id.empty()
            && frontend_tuner_allocation.rf_flow_id != frontend_tuner_status[tuner_id].rf_flow_id
            && frontend_tuner_allocation.tuner_type != "CHANNELIZER"){
            LOG_DEBUG(RFNoC_ProgrammableDevice_i,
              "allocateCapacity: Requested rf_flow_id '"<<frontend_tuner_allocation.rf_flow_id <<"' does not match tuner[" << tuner_id << "].rf_flow_id ("<<frontend_tuner_status[tuner_id].rf_flow_id<<")");
            continue;
        }

        if(frontend_tuner_allocation.device_control){
            double orig_bw = frontend_tuner_status[tuner_id].bandwidth;
            double orig_cf = frontend_tuner_status[tuner_id].center_frequency;
            double orig_sr = frontend_tuner_status[tuner_id].sample_rate;
            // pre-load frontend_tuner_status values (just in case the request is filled but the values are not populated)
            frontend_tuner_status[tuner_id].bandwidth = frontend_tuner_allocation.bandwidth;
            frontend_tuner_status[tuner_id].center_frequency = frontend_tuner_allocation.center_frequency;
            frontend_tuner_status[tuner_id].sample_rate = frontend_tuner_allocation.sample_rate;
            // device control
            if(!tuner_allocation_ids[tuner_id].control_allocation_id.empty() || !deviceSetTuning(frontend_tuner_allocation, frontend_tuner_status[tuner_id], tuner_id)){
                if (frontend_tuner_status[tuner_id].bandwidth == frontend_tuner_allocation.bandwidth)
                    frontend_tuner_status[tuner_id].bandwidth = orig_bw;
                if (frontend_tuner_status[tuner_id].center_frequency == frontend_tuner_allocation.center_frequency)
                    frontend_tuner_status[tuner_id].center_frequency = orig_cf;
                if (frontend_tuner_status[tuner_id].sample_rate == frontend_tuner_allocation.sample_rate)
                    frontend_tuner_status[tuner_id].sample_rate = orig_sr;
                // either not available or didn't succeed setting tuning, try next tuner
                LOG_DEBUG(RFNoC_ProgrammableDevice_i,
                    "allocateCapacity: Tuner["<<tuner_id<<"] is either not available or didn't succeed while setting tuning ");
                continue;
            }
            tuner_allocation_ids[tuner_id].control_allocation_id = frontend_tuner_allocation.allocation_id;
            allocation_id_to_tuner_id.insert(std::pair<std::string, size_t > (frontend_tuner_allocation.allocation_id, tuner_id));
            frontend_tuner_status[tuner_id].allocation_id_csv = createAllocationIdCsv(tuner_id);
        } else {
            // channelizer allocations must specify device control = true
            if(frontend_tuner_allocation.tuner_type == "CHANNELIZER" || frontend_tuner_allocation.tuner_type == "TX"){
                std::ostringstream eout;
                eout<<frontend_tuner_allocation.tuner_type<<" allocation with device_control=false is invalid.";
                LOG_DEBUG(RFNoC_ProgrammableDevice_i, eout.str());
                return false;
            }
            // listener
            if(tuner_allocation_ids[tuner_id].control_allocation_id.empty() || !listenerRequestValidation(frontend_tuner_allocation, tuner_id)){
                // either not allocated or can't support listener request
                LOG_DEBUG(RFNoC_ProgrammableDevice_i,
                    "allocateCapacity: Tuner["<<tuner_id<<"] is either not available or can not support listener request ");
                continue;
            }
            tuner_allocation_ids[tuner_id].listener_allocation_ids.push_back(frontend_tuner_allocation.allocation_id);
            allocation_id_to_tuner_id.insert(std::pair<std::string, size_t > (frontend_tuner_allocation.allocation_id, tuner_id));
            frontend_tuner_status[tuner_id].allocation_id_csv = createAllocationIdCsv(tuner_id);
            this->assignListener(frontend_tuner_allocation.allocation_id,tuner_allocation_ids[tuner_id].control_allocation_id);
        }
        // if we've reached here, we found an eligible tuner with correct frequency

        // check tolerances
        // only check when sample_rate was not set to don't care)
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, std::fixed << " allocateCapacity - SR requested: " << frontend_tuner_allocation.sample_rate
                                                                                           << "  SR got: " << frontend_tuner_status[tuner_id].sample_rate);
        if( (frontend::floatingPointCompare(frontend_tuner_allocation.sample_rate,0)!=0) &&
            (frontend::floatingPointCompare(frontend_tuner_status[tuner_id].sample_rate,frontend_tuner_allocation.sample_rate)<0 ||
                    frontend::floatingPointCompare(frontend_tuner_status[tuner_id].sample_rate,frontend_tuner_allocation.sample_rate+frontend_tuner_allocation.sample_rate * frontend_tuner_allocation.sample_rate_tolerance/100.0)>0 )){
            std::ostringstream eout;
            eout<<std::fixed<<"allocateCapacity("<<int(tuner_id)<<"): returned sr "<<frontend_tuner_status[tuner_id].sample_rate<<" does not meet tolerance criteria of "<<frontend_tuner_allocation.sample_rate_tolerance<<" percent";
            LOG_INFO(RFNoC_ProgrammableDevice_i, eout.str());
            throw std::logic_error(eout.str().c_str());
        }
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, std::fixed << " allocateCapacity - BW requested: " << frontend_tuner_allocation.bandwidth
                                                                                           << "  BW got: " << frontend_tuner_status[tuner_id].bandwidth);
        // Only check when bandwidth was not set to don't care
        if( (frontend::floatingPointCompare(frontend_tuner_allocation.bandwidth,0)!=0) &&
            (frontend::floatingPointCompare(frontend_tuner_status[tuner_id].bandwidth,frontend_tuner_allocation.bandwidth)<0 ||
                    frontend::floatingPointCompare(frontend_tuner_status[tuner_id].bandwidth,frontend_tuner_allocation.bandwidth+frontend_tuner_allocation.bandwidth * frontend_tuner_allocation.bandwidth_tolerance/100.0)>0 )){
            std::ostringstream eout;
            eout<<std::fixed<<"allocateCapacity("<<int(tuner_id)<<"): returned bw "<<frontend_tuner_status[tuner_id].bandwidth<<" does not meet tolerance criteria of "<<frontend_tuner_allocation.bandwidth_tolerance<<" percent";
            LOG_INFO(RFNoC_ProgrammableDevice_i, eout.str());
            throw std::logic_error(eout.str().c_str());
        }

        if(frontend_tuner_allocation.device_control){
            // enable tuner after successful allocation
            try {
                enableTuner(tuner_id,true);
            } catch(...){
                std::ostringstream eout;
                eout<<"allocateCapacity: Failed to enable tuner after allocation";
                LOG_INFO(RFNoC_ProgrammableDevice_i, eout.str());
                throw std::logic_error(eout.str().c_str());
            }
        }
        _usageState = updateUsageState();
        return true;
    }
    // if we made it here, we failed to find an available tuner
    std::ostringstream eout;
    eout<<"allocateCapacity: NO AVAILABLE TUNER. Make sure that the device has an initialized frontend_tuner_status";
    LOG_INFO(RFNoC_ProgrammableDevice_i, eout.str());
    throw std::logic_error(eout.str().c_str());
}

void RFNoC_ProgrammableDevice_i::frontend_tuner_allocation_dealloc(const frontend_tuner_allocation_struct &newDeallocation)
{
    // Try to remove control of the device
    long tuner_id = getTunerMapping(frontend_tuner_allocation.allocation_id);
    if (tuner_id < 0){
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "ALLOCATION_ID NOT FOUND: [" << frontend_tuner_allocation.allocation_id <<"]");
        return;
    }

    if(tuner_allocation_ids[tuner_id].control_allocation_id == frontend_tuner_allocation.allocation_id){
        enableTuner(tuner_id, false);
        removeTunerMapping(tuner_id);
        frontend_tuner_status[tuner_id].allocation_id_csv = createAllocationIdCsv(tuner_id);
    }
    else {
        // send EOS to listener connection only
        removeTunerMapping(tuner_id,frontend_tuner_allocation.allocation_id);
        frontend_tuner_status[tuner_id].allocation_id_csv = createAllocationIdCsv(tuner_id);
    }
}

