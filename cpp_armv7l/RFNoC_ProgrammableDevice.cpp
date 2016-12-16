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
    DEFAULT_BITFILE_PATH("/usr/share/uhd/images/usrp_e310_fpga.bit"),
    HARDWARE_ID("E310"),
    IDLE_BITFILE_PATH("/usr/share/uhd/images/usrp_e3xx_fpga_idle.bit"),
    canUnlink(true)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

RFNoC_ProgrammableDevice_i::RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev) :
    RFNoC_ProgrammableDevice_prog_base_type(devMgr_ior, id, lbl, sftwrPrfl, compDev),
    DEFAULT_BITFILE_PATH("/usr/share/uhd/images/usrp_e310_fpga.bit"),
    HARDWARE_ID("E310"),
    IDLE_BITFILE_PATH("/usr/share/uhd/images/usrp_e3xx_fpga_idle.bit"),
    canUnlink(true)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

RFNoC_ProgrammableDevice_i::RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities) :
    RFNoC_ProgrammableDevice_prog_base_type(devMgr_ior, id, lbl, sftwrPrfl, capacities),
    DEFAULT_BITFILE_PATH("/usr/share/uhd/images/usrp_e310_fpga.bit"),
    HARDWARE_ID("E310"),
    IDLE_BITFILE_PATH("/usr/share/uhd/images/usrp_e3xx_fpga_idle.bit"),
    canUnlink(true)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

RFNoC_ProgrammableDevice_i::RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev) :
    RFNoC_ProgrammableDevice_prog_base_type(devMgr_ior, id, lbl, sftwrPrfl, capacities, compDev),
    DEFAULT_BITFILE_PATH("/usr/share/uhd/images/usrp_e310_fpga.bit"),
    HARDWARE_ID("E310"),
    IDLE_BITFILE_PATH("/usr/share/uhd/images/usrp_e3xx_fpga_idle.bit"),
    canUnlink(true)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

RFNoC_ProgrammableDevice_i::~RFNoC_ProgrammableDevice_i()
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

void RFNoC_ProgrammableDevice_i::constructor()
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Set the load requests and statuses pointers to the properties
    setHwLoadRequestsPtr(&hw_load_requests);
    setHwLoadStatusesPtr(&hw_load_statuses);

    // Set the usrp address
    this->usrpAddress["no_reload_fpga"] = true;

    if (not this->target_device.name.empty()) {
        this->usrpAddress["name"] = this->target_device.name;
    }

    if (not this->target_device.serial.empty()) {
        this->usrpAddress["serial"] = this->target_device.serial;
    }

    if (not this->target_device.type.empty()) {
        this->usrpAddress["type"] = this->target_device.type;
    }

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Trying target_device: " << this->usrpAddress.to_pp_string());

    // Check if the default image path exists. If so, then only personas with a
    // bitfile path that matches the default can be loaded
    if (boost::filesystem::exists(this->DEFAULT_BITFILE_PATH)) {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "*********************************************************************************************************************************************************");
        LOG_INFO(RFNoC_ProgrammableDevice_i, "Default bitfile path (" << this->DEFAULT_BITFILE_PATH << ") exists");
        this->canUnlink = false;

        if (boost::filesystem::is_symlink(this->DEFAULT_BITFILE_PATH)) {
            LOG_WARN(RFNoC_ProgrammableDevice_i, "Default bitfile is a symbolic link. If this is unintentional, please release this Device, unlink the file, and restart the Device");
        } else {
            LOG_WARN(RFNoC_ProgrammableDevice_i, "Due to the presence of this file, only a Persona with a matching FPGA bitfile path can be launched. For proper behavior, please consult the documentation");
        }

        LOG_INFO(RFNoC_ProgrammableDevice_i, "*********************************************************************************************************************************************************");
    }

    // Register the property change listeners
    this->addPropertyListener(this->connectionTable, this, &RFNoC_ProgrammableDevice_i::connectionTableChanged);
    this->addPropertyListener(this->target_device, this, &RFNoC_ProgrammableDevice_i::target_deviceChanged);

    // Set the usage state to IDLE
    setUsageState(CF::Device::IDLE);
}

CF::ExecutableDevice::ProcessID_Type RFNoC_ProgrammableDevice_i::execute (const char* name, const CF::Properties& options, const CF::Properties& parameters)
    throw ( CF::ExecutableDevice::ExecuteFail, CF::InvalidFileName, CF::ExecutableDevice::InvalidOptions,
            CF::ExecutableDevice::InvalidParameters, CF::ExecutableDevice::InvalidFunction, CF::Device::InvalidState,
            CORBA::SystemException)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Call the parent execute
    CF::ExecutableDevice::ProcessID_Type pid = RFNoC_ProgrammableDevice_prog_base_type::execute(name, options, parameters);

    // Map the PID to the device identifier
    std::string deviceIdentifier;

    for (size_t i = 0; i < parameters.length(); ++i) {
        std::string id = parameters[i].id._ptr;

        if (id == "DEVICE_IDENTIFIER") {
            deviceIdentifier = ossie::any_to_string(parameters[i].value);
            break;
        }
    }

    this->pidToDeviceID[pid] = deviceIdentifier;

    return pid;
}

void RFNoC_ProgrammableDevice_i::terminate (CF::ExecutableDevice::ProcessID_Type processId)
                    throw ( CF::Device::InvalidState, CF::ExecutableDevice::InvalidProcess, CORBA::SystemException)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    if (this->pidToDeviceID.find(processId) == this->pidToDeviceID.end()) {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to terminate a process with an ID not tracked by this Device");
        throw CF::ExecutableDevice::InvalidProcess();
    }

    // Get the device identifier associated with this PID
    std::string deviceID = this->pidToDeviceID[processId];

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Terminating device: " << deviceID);

    // Unmap the PID from the device identifier
    this->pidToDeviceID.erase(processId);

    // Unmap the device identifier from the HW Load Status
    this->deviceIDToHwStatus.erase(deviceID);

    // Adjust the HW load statuses property
    this->hw_load_statuses.clear();

    for (deviceHwStatusMap::iterator it = this->deviceIDToHwStatus.begin(); it != this->deviceIDToHwStatus.end(); ++it) {
        this->hw_load_statuses.push_back(it->second);
    }

    // Call the parent terminate
    RFNoC_ProgrammableDevice_prog_base_type::terminate(processId);
}

void RFNoC_ProgrammableDevice_i::releaseObject() throw (CF::LifeCycle::ReleaseError, CORBA::SystemException)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Load the idle bitfile
    loadBitfile(this->IDLE_BITFILE_PATH);

    // Remove the symbolic link, if necessary
    if (this->canUnlink) {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Removing symbolic link from requested bitfile path");

        if (not boost::filesystem::remove(this->DEFAULT_BITFILE_PATH)) {
            LOG_ERROR(RFNoC_ProgrammableDevice_i, "A problem occurred while unlinking the default bitfile symbolic link");
        }
    }

    RFNoC_ProgrammableDevice_prog_base_type::releaseObject();
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

void RFNoC_ProgrammableDevice_i::setHwLoadStatus(const std::string &deviceID, const hw_load_status_object &hwLoadStatus)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    hw_load_statuses_struct_struct hwLoadStatusStruct;
    hwLoadStatusStruct.hardware_id = hwLoadStatus.hardware_id;
    hwLoadStatusStruct.load_filepath = hwLoadStatus.load_filepath;
    hwLoadStatusStruct.request_id = hwLoadStatus.request_id;
    hwLoadStatusStruct.requester_id = hwLoadStatus.requester_id;
    hwLoadStatusStruct.state = hwLoadStatus.state;

    this->deviceIDToHwStatus[deviceID] = hwLoadStatusStruct;

    this->hw_load_statuses.clear();

    for (deviceHwStatusMap::iterator it = this->deviceIDToHwStatus.begin(); it != this->deviceIDToHwStatus.end(); ++it) {
        this->hw_load_statuses.push_back(it->second);
    }
}

Device_impl* RFNoC_ProgrammableDevice_i::generatePersona(int argc, char* argv[], ConstructorPtr personaEntryPoint, const char* libName)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    connectRadioRXCallback connectionRadioRXCb = boost::bind(&RFNoC_ProgrammableDevice_i::connectRadioRX, this, _1, _2, _3);
    connectRadioTXCallback connectionRadioTXCb = boost::bind(&RFNoC_ProgrammableDevice_i::connectRadioTX, this, _1, _2, _3);
    getUsrpCallback getUsrpCb = boost::bind(&RFNoC_ProgrammableDevice_i::getUsrp, this);
    hwLoadStatusCallback hwLoadStatusCb = boost::bind(&RFNoC_ProgrammableDevice_i::setHwLoadStatus, this, _1, _2);

    // Generate the Persona Device
    Device_impl *persona = personaEntryPoint(argc, argv, this, connectionRadioRXCb, connectionRadioTXCb, getUsrpCb, hwLoadStatusCb);

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
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Create a symbolic link from the default file path to the requested file
    // path, if necessary
    if (this->canUnlink) {
        if (boost::filesystem::exists(this->DEFAULT_BITFILE_PATH)) {
            LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Default bitfile path exists, unlinking");

            if (not boost::filesystem::remove(this->DEFAULT_BITFILE_PATH)) {
                LOG_ERROR(RFNoC_ProgrammableDevice_i, "A problem occurred while unlinking the default bitfile symbolic link");
                return false;
            }
        }

        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Creating symbolic link to requested bitfile path");

        boost::system::error_code ec;
        boost::filesystem::create_symlink(requestStatus.load_filepath, this->DEFAULT_BITFILE_PATH, ec);

        if (ec != boost::system::errc::success) {
            LOG_ERROR(RFNoC_ProgrammableDevice_i, "A problem occurred while linking the requested bitfile path");
            return false;
        }
    }

    // Load the requested bitfile
    loadBitfile(this->DEFAULT_BITFILE_PATH);

    // Allow some time for setup
    boost::this_thread::sleep(boost::posix_time::seconds(1.0));

    // Attempt to get a reference to the specified device
    try {
        this->usrp = uhd::device3::make(this->usrpAddress);
    } catch(uhd::key_error &e) {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Unable to find a suitable USRP Device 3.");
        LOG_ERROR(RFNoC_ProgrammableDevice_i, e.what());
        return false;
    } catch(std::exception &e) {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "An error occurred attempting to get a reference to the USRP device.");
        LOG_ERROR(RFNoC_ProgrammableDevice_i, e.what());
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

    // Clear the radio
    if (this->radio.get()) {
        this->radio->clear();
    }

    // Reset the radio pointer
    this->radio.reset();

    // Clear the USRP device
    if (this->usrp.get()) {
        this->usrp->clear();
    }

    // Reset the USRP pointer
    this->usrp.reset();

    // Clear the graph
    this->radioChainGraph.reset();

    // Clear the map of allocation IDs to DDCs/DUCs
    this->allocationIDToDDC.clear();
    this->allocationIDToDUC.clear();

    // Clear the map of radio channels to DDCs/DUCs
    this->radioChannelToDDC.clear();
    this->radioChannelToDUC.clear();

    // Clear the map of tuner IDs to radios
    this->tunerIDToRadioChannel.clear();

    // Clear the map of tuner IDs to radio use status
    this->tunerIDUsed.clear();

    // Clear the frontend_tuner_status and related lists
    setNumChannels(0);
    this->updateSRI.clear();

    // Load the idle bitfile
    loadBitfile(this->IDLE_BITFILE_PATH);

    // Remove the symbolic link, if necessary
    if (this->canUnlink) {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Removing symbolic link from requested bitfile path");

        if (not boost::filesystem::remove(this->DEFAULT_BITFILE_PATH)) {
            LOG_ERROR(RFNoC_ProgrammableDevice_i, "A problem occurred while unlinking the default bitfile symbolic link");
        }
    }
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

    if (not this->canUnlink) {
        if (hwLoadRequestStruct.load_filepath != this->DEFAULT_BITFILE_PATH) {
            LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to validate hardware load request. The default bitfile exists and the requested bitfile does not match.");
            LOG_WARN(RFNoC_ProgrammableDevice_i, "Default bitfile: " << this->DEFAULT_BITFILE_PATH);
            LOG_WARN(RFNoC_ProgrammableDevice_i, "Received bitfile: " << hwLoadRequestStruct.load_filepath);
            return false;
        }
    }

    return true;
}

void RFNoC_ProgrammableDevice_i::initializeRadioChain()
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Grab the radio blocks
    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Getting radio block");
    this->radio = this->usrp->get_block_ctrl<uhd::rfnoc::radio_ctrl>("Radio");

    if (not this->radio.get()) {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Unable to get the Radio block for this hardware load");
        return;
    }

    // Grab the DDC blocks
    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Searching for DDC blocks");
    std::vector<uhd::rfnoc::block_id_t> ddcBlockIDs = this->usrp->find_blocks("DDC");
    std::vector<uhd::rfnoc::ddc_block_ctrl::sptr> tmpDdcs;

    size_t numDdcChannels = 0;

    for (size_t i = 0; i < ddcBlockIDs.size(); ++i) {
        uhd::rfnoc::block_id_t blockId = ddcBlockIDs[i];

        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Found DDC block with ID: " << blockId.to_string());

        uhd::rfnoc::ddc_block_ctrl::sptr ddc = this->usrp->get_block_ctrl<uhd::rfnoc::ddc_block_ctrl>(blockId);

        numDdcChannels += ddc->get_input_ports().size();

        tmpDdcs.push_back(ddc);
    }

    // Grab the DUC blocks
    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Searching for DUC blocks");
    std::vector<uhd::rfnoc::block_id_t> ducBlockIDs = this->usrp->find_blocks("DUC");
    std::vector<uhd::rfnoc::duc_block_ctrl::sptr> tmpDucs;

    size_t numDucChannels = 0;

    for (size_t i = 0; i < ducBlockIDs.size(); ++i) {
        uhd::rfnoc::block_id_t blockId = ducBlockIDs[i];

        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Found DUC block with ID: " << blockId.to_string());

        uhd::rfnoc::duc_block_ctrl::sptr duc = this->usrp->get_block_ctrl<uhd::rfnoc::duc_block_ctrl>(blockId);

        numDucChannels += duc->get_input_ports();

        tmpDucs.push_back(duc);
    }

    // Connect the radio to the DDC(s) and DUC(s), if possible
    if (numDdcChannels == 0) {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "No DDCs available, RX not possible");
    } else if (numDdcChannels == 1) {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "Only one DDC channel available, RX possible on one channel only")

        this->radioChainGraph->connect(this->radio->get_block_id(), tmpDdcs[0]->get_block_id());

        this->radioChannelToDDC[0] = std::make_pair(tmpDdcs[0], 0);
    } else {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "Sufficient DDCs for RX on all radios");

        if (tmpDdcs.size() == 1) {
            this->radioChainGraph->connect(this->radio->get_block_id(), 0, tmpDdcs[0]->get_block_id(), 0);
            this->radioChainGraph->connect(this->radio->get_block_id(), 1, tmpDdcs[0]->get_block_id(), 1);

            this->radioChannelToDDC[0] = std::make_pair(tmpDdcs[0], 0);
            this->radioChannelToDDC[1] = std::make_pair(tmpDdcs[0], 1);
        } else {
            this->radioChainGraph->connect(this->radio->get_block_id(), 0, tmpDdcs[0]->get_block_id(), 0);
            this->radioChainGraph->connect(this->radio->get_block_id(), 1, tmpDdcs[1]->get_block_id(), 0);

            this->radioChannelToDDC[0] = std::make_pair(tmpDdcs[0], 0);
            this->radioChannelToDDC[1] = std::make_pair(tmpDdcs[1], 0);
        }
    }

    if (numDucChannels == 0) {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "No DUCs available, TX not possible");
    } else if (numDucChannels == 1) {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "Only one DUC channel available, TX possible on one channel only")

        this->radioChainGraph->connect(tmpDucs[0]->get_block_id(), this->radio->get_block_id());

        this->radioChannelToDUC[0] = std::make_pair(tmpDucs[0], 0);
    } else {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "Sufficient DUCs for TX on all radios");

        if (tmpDucs.size() == 1) {
            this->radioChainGraph->connect(tmpDucs[0]->get_block_id(), 0, this->radio->get_block_id(), 0);
            this->radioChainGraph->connect(tmpDucs[0]->get_block_id(), 1, this->radio->get_block_id(), 1);

            this->radioChannelToDUC[0] = std::make_pair(tmpDucs[0], 0);
            this->radioChannelToDUC[1] = std::make_pair(tmpDucs[0], 1);
        } else {
            this->radioChainGraph->connect(tmpDucs[0]->get_block_id(), 0, this->radio->get_block_id(), 0);
            this->radioChainGraph->connect(tmpDucs[1]->get_block_id(), 0, this->radio->get_block_id(), 1);

            this->radioChannelToDUC[0] = std::make_pair(tmpDucs[0], 0);
            this->radioChannelToDUC[1] = std::make_pair(tmpDucs[1], 0);
        }
    }

    setNumChannels(numDdcChannels + numDucChannels);

    size_t currentStatus = 0;

    std::vector<size_t> outputPorts = this->radio->get_output_ports();

    for (size_t i = 0; i < outputPorts.size(); ++i) {
        frontend_tuner_status_struct_struct &fts = this->frontend_tuner_status[currentStatus];

        fts.bandwidth = this->radio->get_output_samp_rate(i);
        fts.center_frequency = this->radio->get_rx_frequency(i);
        fts.sample_rate = this->radio->get_output_samp_rate(i);

        this->tunerIDToRadioChannel[currentStatus] = i;
        this->tunerIDUsed[currentStatus] = false;

        ++currentStatus;
    }

    std::vector<size_t> inputPorts = this->radio->get_input_ports();

    for (size_t i = 0; i < inputPorts.size(); ++i) {
        frontend_tuner_status_struct_struct &fts = this->frontend_tuner_status[currentStatus];

        fts.bandwidth = this->radio->get_input_samp_rate(i);
        fts.center_frequency = this->radio->get_tx_frequency(i);
        fts.sample_rate = this->radio->get_input_samp_rate(i);
        fts.tuner_type = "TX";

        this->tunerIDToRadioChannel[currentStatus] = i;
        this->tunerIDUsed[currentStatus] = false;

        ++currentStatus;
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

    size_t radioChannel = this->tunerIDToRadioChannel[tuner_id];

    if (fts.tuner_type == "RX_DIGITIZER") {
        radio->set_rx_streamer(true, radioChannel);
    } else if (fts.tuner_type == "TX") {
        radio->set_tx_streamer(true, radioChannel);
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

    size_t radioChannel = this->tunerIDToRadioChannel[tuner_id];

    if (fts.tuner_type == "RX_DIGITIZER") {
        radio->set_rx_streamer(false, radioChannel);
    } else if (fts.tuner_type == "TX") {
        radio->set_tx_streamer(false, radioChannel);
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
    size_t radioChannel = this->tunerIDToRadioChannel.find(tuner_id);

    // Make sure it isn't already in use
    if (this->tunerIDUsed[tuner_id]) {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Failed to set tuning: Requested tuner already in use");
        return false;
    }

    // Attempt to set the radio with the requested values
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
            actualCF = this->radio->set_rx_frequency(request.center_frequency, radioChannel);

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
            actualCF = this->radio->set_tx_frequency(request.center_frequency, radioChannel);

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
        std::pair<uhd::rfnoc::ddc_block_ctrl::sptr, size_t> ddc = this->radioChannelToDDC[radioChannel];

        this->allocationIDToDDC[request.allocation_id] = ddc;
    } else if (request.tuner_type == "TX") {
        std::pair<uhd::rfnoc::duc_block_ctrl::sptr, size_t> duc = this->radioChannelToDUC[radioChannel];

        this->allocationIDToDUC[request.allocation_id] = duc;
    }

    // Mark this radio as used
    this->tunerIDUsed[tuner_id];

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Allocation succeeded on: " << this->radio->get_block_id().to_string() << ":" << radioChannel);

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

    // No need to clean up if the tuner was not in use
    if (not this->tunerIDUsed[tuner_id]) {
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

bool RFNoC_ProgrammableDevice_i::loadBitfile(const std::string &bitfilePath)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Attempting to load bitfile: " << bitfilePath);

    uhd::image_loader::image_loader_args_t image_loader_args;

    image_loader_args.firmware_path = "";
    image_loader_args.fpga_path = bitfilePath;
    image_loader_args.load_firmware = false;
    image_loader_args.load_fpga = true;

    return uhd::image_loader::load(image_loader_args);
}
