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

    // Register the property change listeners
    this->addPropertyListener(this->connectionTable, this, &RFNoC_ProgrammableDevice_i::connectionTableChanged);
    this->addPropertyListener(this->target_device, this, &RFNoC_ProgrammableDevice_i::target_deviceChanged);

    // Set the usage state to IDLE
    setUsageState(CF::Device::IDLE);
}

bool RFNoC_ProgrammableDevice_i::connectRadioRX(const CORBA::ULong &portHash, const uhd::rfnoc::block_id_t &blockToConnect, const size_t &blockPort)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    if (not this->radioChainGraph.get()) {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Unable to connect radio without graph");
        return false;
    }

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Checking output port for hash " << portHash << " to connecto radio chain to " << blockToConnect.to_string());

    bulkio::OutShortPort::ConnectionsList connections = this->dataShort_out->getConnections();

    for (size_t i = 0; i < connections.size(); ++i) {
        CORBA::ULong providesHash = connections[i].first->_hash(1024);

        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Checking hash " << providesHash);

        if (providesHash == portHash) {
            LOG_INFO(RFNoC_ProgrammableDevice_i, "Found correct connection, retrieving DDC information");
            std::string connectionID = connections[i].second;

            std::map<std::string, std::pair<uhd::rfnoc::ddc_block_ctrl::sptr, size_t> >::iterator it = this->allocationIDToDDC.find(connectionID);

            if (it == this->allocationIDToDDC.end()) {
                LOG_WARN(RFNoC_ProgrammableDevice_i, "Unable to find DDC for allocation/connection ID: " << connectionID);
                continue;
            }

            uhd::rfnoc::ddc_block_ctrl::sptr ddc = it->second.first;
            size_t ddcPort = it->second.second;

            this->radioChainGraph->connect(ddc->get_block_id(), ddcPort, blockToConnect, blockPort);

            return true;
        }
    }

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "No connection possible");

    return false;
}

bool RFNoC_ProgrammableDevice_i::connectRadioTX(const std::string &allocationID, const uhd::rfnoc::block_id_t &blockToConnect, const size_t &blockPort)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    if (not this->radioChainGraph.get()) {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Unable to connect radio without graph");
        return false;
    }

    std::map<std::string, std::pair<uhd::rfnoc::duc_block_ctrl::sptr, size_t> >::iterator it = this->allocationIDToDUC.find(allocationID);

    if (it == this->allocationIDToDUC.end()) {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to connect to DUC with unknown allocation ID: " << allocationID);
        return false;
    }

    uhd::rfnoc::duc_block_ctrl::sptr duc = it->second.first;
    size_t ducPort = it->second.second;

    this->radioChainGraph->connect(blockToConnect, blockPort, duc->get_block_id(), ducPort);

    return true;
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

    hwLoadStatusCallback hwLoadStatusCb = boost::bind(&RFNoC_ProgrammableDevice_i::setHwLoadStatus, this, _1);
    connectRadioRXCallback connectionRadioRXCb = boost::bind(&RFNoC_ProgrammableDevice_i::connectRadioRX, this, _1, _2, _3);
    connectRadioTXCallback connectionRadioTXCb = boost::bind(&RFNoC_ProgrammableDevice_i::connectRadioTX, this, _1, _2, _3);

    // Generate the Persona Device
    Device_impl *persona = personaEntryPoint(argc, argv, this, hwLoadStatusCb, connectionRadioRXCb, connectionRadioTXCb, this->usrpAddress);

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

    // Create the RF-NoC graph
    this->radioChainGraph = this->usrp->create_graph("radioChainGraph");

    // Attempt to get the radios, DDCs, and DUCs
    initializeRadioChain();

    return true;
}

void RFNoC_ProgrammableDevice_i::unloadHardware(const HwLoadStatusStruct& requestStatus) 
{
    // The hardware may be physically unloaded at this point
    LOG_INFO(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Reset the USRP device
    if (this->usrp.get()) {
        this->usrp->clear();
    }

    // Load the idle bitfile
    uhd::image_loader::image_loader_args_t image_loader_args;

    image_loader_args.firmware_path = "";
    image_loader_args.fpga_path = this->IDLE_BITFILE_PATH;
    image_loader_args.load_firmware = false;
    image_loader_args.load_fpga = true;

    uhd::image_loader::load(image_loader_args);

    // Clear the USRP pointer
    this->usrp.reset();

    // Clear the graph
    this->radioChainGraph.reset();

    // Clear the map of allocation IDs to DDCs/DUCs
    this->allocationIDToDDC.clear();
    this->allocationIDToDUC.clear();

    // Clear the map of radio IDs to DDCs/DUCs
    this->radioIDToDDC.clear();
    this->radioIDToDUC.clear();

    // Clear the map of tuner IDs to radios
    this->tunerIDToRadio.clear();

    // Clear the map of tuner IDs to radio use status
    this->tunerIDUsed.clear();

    // Clear the radios, DDCs, and DUCs
    this->radios.clear();
    this->ddcs.clear();
    this->ducs.clear();

    // Clear the frontend_tuner_status and related lists
    setNumChannels(0);
    this->updateSRI.clear();
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

    // Grab the radio blocks
    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Searching for radio blocks");
    std::vector<uhd::rfnoc::block_id_t> radioBlockIDs = this->usrp->find_blocks("Radio");

    for (size_t i = 0; i < radioBlockIDs.size(); ++i) {
        uhd::rfnoc::block_id_t blockId = radioBlockIDs[i];

        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Found radio block with ID: " << blockId.to_string());

        uhd::rfnoc::radio_ctrl::sptr radio = this->usrp->get_block_ctrl<uhd::rfnoc::radio_ctrl>(blockId);

        this->radios.push_back(radio);
    }

    // Grab the DDC blocks
    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Searching for DDC blocks");
    std::vector<uhd::rfnoc::block_id_t> ddcBlockIDs = this->usrp->find_blocks("DDC");

    for (size_t i = 0; i < ddcBlockIDs.size(); ++i) {
        uhd::rfnoc::block_id_t blockId = ddcBlockIDs[i];

        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Found DDC block with ID: " << blockId.to_string());

        uhd::rfnoc::ddc_block_ctrl::sptr ddc = this->usrp->get_block_ctrl<uhd::rfnoc::ddc_block_ctrl>(blockId);

        this->ddcs.push_back(ddc);
    }

    // Grab the DUC blocks
    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Searching for DUC blocks");
    std::vector<uhd::rfnoc::block_id_t> ducBlockIDs = this->usrp->find_blocks("DUC");

    for (size_t i = 0; i < ducBlockIDs.size(); ++i) {
        uhd::rfnoc::block_id_t blockId = ducBlockIDs[i];

        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Found DUC block with ID: " << blockId.to_string());

        uhd::rfnoc::duc_block_ctrl::sptr duc = this->usrp->get_block_ctrl<uhd::rfnoc::duc_block_ctrl>(blockId);

        this->ducs.push_back(duc);
    }

    // Determine the number of valid chains
    size_t validRXChains = std::min(this->radios.size(), this->ddcs.size());
    size_t validTXChains = std::min(this->radios.size(), this->ducs.size());

    if (validRXChains != this->radios.size()) {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Not enough DDCs are available for the number of radios");
    }

    if (validTXChains != this->radios.size()) {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Not enough DUCs are available for the number of radios");
    }

    // Determine the number of channels
    size_t numRXChannels = 0;
    size_t numTXChannels = 0;

    // Iterate over the valid RX chains, gathering the number of channels and
    // connecting the radio blocks to the DDC blocks
    for (size_t i = 0; i < validRXChains; ++i) {
        uhd::rfnoc::block_id_t ddcBlockID = this->ddcs[i]->get_block_id();
        std::vector<size_t> outputPorts = this->radios[i]->get_output_ports();
        uhd::rfnoc::block_id_t radioBlockID = this->radios[i]->get_block_id();

        numRXChannels += outputPorts.size();

        for (size_t j = 0; j < outputPorts.size(); ++j) {
            size_t portNumber = outputPorts[j];

            LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Connecting port " << portNumber << " between " << radioBlockID.to_string() << " and " << ddcBlockID.to_string());

            this->radioChainGraph->connect(radioBlockID, portNumber, ddcBlockID, portNumber);

            std::stringstream modifiedRadioID;

            modifiedRadioID << this->radios[i]->get_block_id().to_string();
            modifiedRadioID << j;

            this->radioIDToDDC[modifiedRadioID.str()] = std::make_pair(this->ddcs[i], j);
        }
    }

    // Iterate over the valid TX chains, gathering the number of channels and
    // connecting the radio blocks to the DUC blocks
    for (size_t i = 0; i < validTXChains; ++i) {
        uhd::rfnoc::block_id_t ducBlockID = this->ducs[i]->get_block_id();
        std::vector<size_t> inputPorts = this->radios[i]->get_input_ports();
        uhd::rfnoc::block_id_t radioBlockID = this->radios[i]->get_block_id();

        numTXChannels += inputPorts.size();

        for (size_t j = 0; j < inputPorts.size(); ++j) {
            size_t portNumber = inputPorts[j];

            LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Connecting port " << portNumber << " between " << ducBlockID.to_string() << " and " << radioBlockID.to_string());

            this->radioChainGraph->connect(ducBlockID, portNumber, radioBlockID, portNumber);

            std::stringstream modifiedRadioID;

            modifiedRadioID << this->radios[i]->get_block_id().to_string();
            modifiedRadioID << j;

            this->radioIDToDUC[modifiedRadioID.str()] = std::make_pair(this->ducs[i], j);
        }
    }

    setNumChannels(numRXChannels + numTXChannels);

    size_t currentStatus = 0;

    for (size_t i = 0; i < validRXChains; ++i) {
        std::vector<size_t> outputPorts = this->radios[i]->get_output_ports();

        for (size_t j = 0; j < outputPorts.size(); ++j) {
            frontend_tuner_status_struct_struct &fts = this->frontend_tuner_status[currentStatus];

            fts.bandwidth = this->radios[i]->get_output_samp_rate(j);
            fts.center_frequency = this->radios[i]->get_rx_frequency(j);
            fts.sample_rate = this->radios[i]->get_output_samp_rate(j);

            this->tunerIDToRadio[currentStatus] = std::make_pair(this->radios[i], j);
            this->tunerIDUsed[currentStatus] = false;

            ++currentStatus;
        }
    }

    for (size_t i = 0; i < validTXChains; ++i) {
        std::vector<size_t> inputPorts = this->radios[i]->get_input_ports();

        for (size_t j = 0; j < inputPorts.size(); ++j) {
            frontend_tuner_status_struct_struct &fts = this->frontend_tuner_status[currentStatus];

            fts.bandwidth = this->radios[i]->get_input_samp_rate(j);
            fts.center_frequency = this->radios[i]->get_tx_frequency(j);
            fts.sample_rate = this->radios[i]->get_input_samp_rate(j);
            fts.tuner_type = "TX";

            this->tunerIDToRadio[currentStatus] = std::make_pair(this->radios[i], j);
            this->tunerIDUsed[currentStatus] = false;

            ++currentStatus;
        }
    }

    this->updateSRI.clear();
    this->updateSRI.resize(this->frontend_tuner_status.size(), false);
}

void RFNoC_ProgrammableDevice_i::target_deviceChanged(const target_device_struct &oldValue, const target_device_struct &newValue)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to set the target_device while running. Must be set in DCD file.");

    this->target_device = oldValue;
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

    std::pair<uhd::rfnoc::radio_ctrl::sptr, size_t> radio = this->tunerIDToRadio[tuner_id];

    if (fts.tuner_type == "RX_DIGITIZER") {
        radio.first->set_rx_streamer(true, radio.second);
    } else if (fts.tuner_type == "TX") {
        radio.first->set_tx_streamer(true, radio.second);
    }

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

    std::pair<uhd::rfnoc::radio_ctrl::sptr, size_t> radio = this->tunerIDToRadio[tuner_id];

    if (fts.tuner_type == "RX_DIGITIZER") {
        radio.first->set_rx_streamer(false, radio.second);
    } else if (fts.tuner_type == "TX") {
        radio.first->set_tx_streamer(false, radio.second);
    }

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

    // Get the radio and channel for the requested tuner
    std::map<size_t, std::pair<uhd::rfnoc::radio_ctrl::sptr, size_t> >::iterator it;

    it = this->tunerIDToRadio.find(tuner_id);

    if (it == this->tunerIDToRadio.end()) {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Failed to set tuning: Invalid tuner ID");
        return false;
    }

    // Make sure it isn't already in use
    if (this->tunerIDUsed[tuner_id]) {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Failed to set tuning: Requested tuner already in use");
        return false;
    }

    // Attempt to set the radio with the requested values
    uhd::rfnoc::radio_ctrl::sptr radio = it->second.first;
    size_t channel = it->second.second;

    if (not frontend::validateRequest(16e6, 16e6, request.bandwidth)) {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Failed to set tuning: Bandwidth doesn't match 16 MHz");
        return false;
    }

    if (not frontend::validateRequest(70e6, 6000e6, request.center_frequency)) {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Failed to set tuning: Center frequency not between 70 MHz and 6 GHz");
        return false;
    }

    if (not frontend::validateRequest(16e6, 16e6, request.sample_rate)) {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Failed to set tuning: Sample rate doesn't match 16 MSps");
        return false;
    }

    // Set the center frequency
    double actualCF;

    if (request.tuner_type == "RX_DIGITIZER") {
        bool succeeded = false;

        try {
            actualCF = radio->set_rx_frequency(request.center_frequency, channel);

            LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Requested CF: " << request.center_frequency << ", Actual CF: " << actualCF);

            succeeded = true;
        } catch(...) {

        }

        if (not succeeded) {
            LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to set tuning: Unable to set center frequency");
            return false;
        }
    } else if (request.tuner_type == "TX") {
        bool succeeded = false;

        try {
            actualCF = radio->set_tx_frequency(request.center_frequency, channel);

            LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Requested CF: " << request.center_frequency << ", Actual CF: " << actualCF);

            succeeded = true;
        } catch(...) {

        }

        if (not succeeded) {
            LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to set tuning: Unable to set center frequency");
            return false;
        }
    } else {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to set tuning: Invalid tuner type");
        return false;
    }

    // Set the frontend tuner status
    fts.bandwidth = 16e6;
    fts.center_frequency = actualCF;
    fts.sample_rate = 16e6;

    // Map the allocation ID to the DDC or DUC
    if (request.tuner_type == "RX_DIGITIZER") {
        std::stringstream modifiedRadioID;

        modifiedRadioID << radio->get_block_id().to_string();
        modifiedRadioID << channel;

        std::pair<uhd::rfnoc::ddc_block_ctrl::sptr, size_t> ddc = this->radioIDToDDC[modifiedRadioID.str()];

        this->allocationIDToDDC[request.allocation_id] = ddc;
    } else if (request.tuner_type == "TX") {
        std::stringstream modifiedRadioID;

        modifiedRadioID << radio->get_block_id().to_string();
        modifiedRadioID << channel;

        std::pair<uhd::rfnoc::duc_block_ctrl::sptr, size_t> duc = this->radioIDToDUC[modifiedRadioID.str()];

        this->allocationIDToDUC[request.allocation_id] = duc;
    }

    // Mark this radio as used
    this->tunerIDUsed[tuner_id];

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Allocation succeeded on: " << radio->get_block_id().to_string() << ":" << channel);

    // creates a stream id if not already created for this tuner
    std::string stream_id = getStreamId(tuner_id);

    // enable multi-out capability for this stream/allocation/connection
    matchAllocationIdToStreamId(request.allocation_id, stream_id, "dataShort_out");

    this->updateSRI[tuner_id] = true;

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

    std::map<size_t, bool>::iterator it = this->tunerIDUsed.find(tuner_id);

    if (it == this->tunerIDUsed.end()) {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Failed to set tuning: Invalid tuner ID");
        return false;
    }

    // No need to clean up if the tuner was not in use
    if (not it->second) {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Device with tuner ID: " << tuner_id << " was not in use");
        return false;
    }

    // Get the control allocation ID
    const std::string allocationId = getControlAllocationId(tuner_id);

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Found allocation ID for tuner: " << allocationId);

    // Clear the allocation ID to DDC/DUC mapping
    std::map<std::string, std::pair<uhd::rfnoc::ddc_block_ctrl::sptr, size_t> >::iterator ddcIt = this->allocationIDToDDC.find(allocationId);

    if (ddcIt != this->allocationIDToDDC.end()) {
        this->allocationIDToDDC.erase(ddcIt);
    }

    std::map<std::string, std::pair<uhd::rfnoc::duc_block_ctrl::sptr, size_t> >::iterator ducIt = this->allocationIDToDUC.find(allocationId);

    if (ducIt != this->allocationIDToDUC.end()) {
        this->allocationIDToDUC.erase(ducIt);
    }

    // Clear the tuner in use flag
    this->tunerIDUsed[tuner_id] = false;

    return true;
}

std::string RFNoC_ProgrammableDevice_i::getStreamId(size_t tuner_id)
{
    if (tuner_id >= this->frontend_tuner_status.size()) {
        return "ERR: INVALID TUNER ID";
    }

    if (this->frontend_tuner_status[tuner_id].stream_id.empty()){
        std::ostringstream id;
        id<<"tuner_freq_"<<long(this->frontend_tuner_status[tuner_id].center_frequency)<<"_Hz_"<<frontend::uuidGenerator();
        this->frontend_tuner_status[tuner_id].stream_id = id.str();
        this->updateSRI[tuner_id] = true;
        LOG_DEBUG(RFNoC_ProgrammableDevice_i,"RFNoC_ProgrammableDevice_i::getStreamId - created NEW stream id: "<< frontend_tuner_status[tuner_id].stream_id);
    } else {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i,"RFNoC_ProgrammableDevice_i::getStreamId - returning EXISTING stream id: "<< frontend_tuner_status[tuner_id].stream_id);
    }
    return frontend_tuner_status[tuner_id].stream_id;
}
