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

    // Register the connection listeners
    this->dataShort_out->setNewConnectListener(this, &RFNoC_ProgrammableDevice_i::connectionAdded);
    this->dataShort_out->setNewDisconnectListener(this, &RFNoC_ProgrammableDevice_i::connectionRemoved);

    // Set the usage state to IDLE
    setUsageState(CF::Device::IDLE);

    // Start the Device
    start();
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

        if (id == "DEVICE_ID") {
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

    // Unmap the device identifier from the Get Block Info callback
    this->deviceIDToGetBlockInfo.erase(deviceID);

    // Unmap the device identifier from the HW Load Status
    this->deviceIDToHwStatus.erase(deviceID);

    // Unmap the PID from the device identifier
    this->pidToDeviceID.erase(processId);

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

            std::map<std::string, RxObject *>::iterator it = this->allocationIDToRx.find(connectionID);

            if (it == this->allocationIDToRx.end()) {
                LOG_WARN(RFNoC_ProgrammableDevice_i, "Unable to find RX object for allocation/connection ID: " << connectionID);
                continue;
            } else if (it->second->connected) {
                LOG_DEBUG(RFNoC_ProgrammableDevice_i, "RX object already connected for allocation/connection ID: " << connectionID);
                return true;
            }

            uhd::rfnoc::ddc_block_ctrl::sptr ddc = it->second->ddc;
            size_t ddcPort = it->second->ddcPort;

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

    std::map<std::string, TxObject *>::iterator it = this->allocationIDToTx.find(allocationID);

    if (it == this->allocationIDToTx.end()) {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to connect to DUC with unknown allocation ID: " << allocationID);
        return false;
    }

    uhd::rfnoc::duc_block_ctrl::sptr duc = it->second->duc;
    size_t ducPort = it->second->ducPort;

    this->radioChainGraph->connect(blockToConnect, blockPort, duc->get_block_id(), ducPort);

    return true;
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

    connectRadioRXCallback connectRadioRXCb = boost::bind(&RFNoC_ProgrammableDevice_i::connectRadioRX, this, _1, _2, _3);
    connectRadioTXCallback connectRadioTXCb = boost::bind(&RFNoC_ProgrammableDevice_i::connectRadioTX, this, _1, _2, _3);
    getUsrpCallback getUsrpCb = boost::bind(&RFNoC_ProgrammableDevice_i::getUsrp, this);
    hwLoadStatusCallback hwLoadStatusCb = boost::bind(&RFNoC_ProgrammableDevice_i::setHwLoadStatus, this, _1, _2);
    setGetBlockInfoFromHashCallback setGetBlockInfoFromHashCb = boost::bind(&RFNoC_ProgrammableDevice_i::setGetBlockInfoFromHash, this, _1, _2);

    // Generate the Persona Device
    Device_impl *persona = personaEntryPoint(argc, argv, this, connectRadioRXCb, connectRadioTXCb, getUsrpCb, hwLoadStatusCb, setGetBlockInfoFromHashCb);

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

    // Attempt to get the radios, DDCs, and DUCs
    initializeRadioChain();

    // Set the active device ID
    this->activeDeviceID = requestStatus.requester_id;

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

    // Clear the flows
    clearFlows();

    // Clear the frontend_tuner_status
    setNumChannels(0);

    // Load the idle bitfile
    loadBitfile(this->IDLE_BITFILE_PATH);

    // Remove the symbolic link, if necessary
    if (this->canUnlink) {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Removing symbolic link from requested bitfile path");

        if (not boost::filesystem::remove(this->DEFAULT_BITFILE_PATH)) {
            LOG_ERROR(RFNoC_ProgrammableDevice_i, "A problem occurred while unlinking the default bitfile symbolic link");
        }
    }

    // Clear the active device ID
    this->activeDeviceID.clear();
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

    // Clear everything
    if (this->radio.get()) {
        this->radio.reset();
    }

    if (this->radioChainGraph.get()) {
        this->radioChainGraph.reset();
    }

    // Clear the flows
    clearFlows();

    // Create the RF-NoC graph
    this->radioChainGraph = this->usrp->create_graph("radioChainGraph");

    // Grab the radio blocks
    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Getting radio block");
    this->radio = this->usrp->get_block_ctrl<uhd::rfnoc::radio_ctrl>(uhd::rfnoc::block_id_t("Radio"));

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

        numDucChannels += duc->get_input_ports().size();

        tmpDucs.push_back(duc);
    }

    // Check the properties
    if (this->desiredRxChannels > numDdcChannels) {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Desired number of RX channels greater than available DDC channels.");
        this->desiredRxChannels = numDdcChannels;
    } else {
        numDdcChannels = this->desiredRxChannels;
    }

    if (this->desiredTxChannels > numDucChannels) {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Desired number of TX channels greater than available DUC channels.");
        this->desiredTxChannels = numDucChannels;
    } else {
        numDucChannels = this->desiredTxChannels;
    }

    // Instantiate the flow objects
    size_t tunerID = 0;

    for (size_t i = 0; i < numDdcChannels; ++i) {
        this->tunerIDToRx[tunerID] = new RxObject;

        this->tunerIDToRx[tunerID]->connected = false;
        this->tunerIDToRx[tunerID]->rxThread = NULL;
        this->tunerIDToRx[tunerID]->streamStarted = false;
        this->tunerIDToRx[tunerID]->updateSRI = false;
        this->tunerIDToRx[tunerID]->used = false;

        ++tunerID;
    }

    for (size_t i = 0; i < numDucChannels; ++i) {
        this->tunerIDToTx[tunerID] = new TxObject;

        this->tunerIDToTx[tunerID]->txThread = NULL;
        this->tunerIDToTx[tunerID]->used = false;

        ++tunerID;
    }

    // Connect the radio to the DDC(s) and DUC(s), if possible
    if (numDdcChannels == 0) {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "No DDCs available, RX not possible");
    } else if (numDdcChannels == 1) {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "Only one DDC channel available, RX possible on one channel only")

        this->radioChainGraph->connect(this->radio->get_block_id(), 0, tmpDdcs[0]->get_block_id(), 0);

        tunerIDToRx[0]->ddc = tmpDdcs[0];
        tunerIDToRx[0]->ddcPort = 0;
        tunerIDToRx[0]->radioChannel = 0;
    } else {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "Sufficient DDCs for RX on all radios");

        if (tmpDdcs.size() == 1) {
            this->radioChainGraph->connect(this->radio->get_block_id(), 0, tmpDdcs[0]->get_block_id(), 0);
            this->radioChainGraph->connect(this->radio->get_block_id(), 1, tmpDdcs[0]->get_block_id(), 1);

            tunerIDToRx[0]->ddc = tmpDdcs[0];
            tunerIDToRx[0]->ddcPort = 0;
            tunerIDToRx[0]->radioChannel = 0;

            tunerIDToRx[1]->ddc = tmpDdcs[0];
            tunerIDToRx[1]->ddcPort = 1;
            tunerIDToRx[1]->radioChannel = 1;
        } else {
            this->radioChainGraph->connect(this->radio->get_block_id(), 0, tmpDdcs[0]->get_block_id(), 0);
            this->radioChainGraph->connect(this->radio->get_block_id(), 1, tmpDdcs[1]->get_block_id(), 0);

            tunerIDToRx[0]->ddc = tmpDdcs[0];
            tunerIDToRx[0]->ddcPort = 0;
            tunerIDToRx[0]->radioChannel = 0;

            tunerIDToRx[1]->ddc = tmpDdcs[1];
            tunerIDToRx[1]->ddcPort = 0;
            tunerIDToRx[1]->radioChannel = 1;
        }
    }

    if (numDucChannels == 0) {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "No DUCs available, TX not possible");
    } else if (numDucChannels == 1) {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "Only one DUC channel available, TX possible on one channel only")

        this->radioChainGraph->connect(tmpDucs[0]->get_block_id(), 0, this->radio->get_block_id(), 0);

        tunerIDToTx[numDdcChannels]->duc = tmpDucs[0];
        tunerIDToTx[numDdcChannels]->ducPort = 0;
        tunerIDToTx[numDdcChannels]->radioChannel = 0;
    } else {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "Sufficient DUCs for TX on all radios");

        if (tmpDucs.size() == 1) {
            this->radioChainGraph->connect(tmpDucs[0]->get_block_id(), 0, this->radio->get_block_id(), 0);
            this->radioChainGraph->connect(tmpDucs[0]->get_block_id(), 1, this->radio->get_block_id(), 1);

            tunerIDToTx[numDdcChannels]->duc = tmpDucs[0];
            tunerIDToTx[numDdcChannels]->ducPort = 0;
            tunerIDToTx[numDdcChannels]->radioChannel = 0;

            tunerIDToTx[numDdcChannels + 1]->duc = tmpDucs[0];
            tunerIDToTx[numDdcChannels + 1]->ducPort = 1;
            tunerIDToTx[numDdcChannels + 1]->radioChannel = 1;
        } else {
            this->radioChainGraph->connect(tmpDucs[0]->get_block_id(), 0, this->radio->get_block_id(), 0);
            this->radioChainGraph->connect(tmpDucs[1]->get_block_id(), 0, this->radio->get_block_id(), 1);

            tunerIDToTx[numDdcChannels]->duc = tmpDucs[0];
            tunerIDToTx[numDdcChannels]->ducPort = 0;
            tunerIDToTx[numDdcChannels]->radioChannel = 0;

            tunerIDToTx[numDdcChannels + 1]->duc = tmpDucs[1];
            tunerIDToTx[numDdcChannels + 1]->ducPort = 0;
            tunerIDToTx[numDdcChannels + 1]->radioChannel = 1;
        }
    }

    setNumChannels(numDdcChannels + numDucChannels);

    size_t currentStatus = 0;

    std::vector<size_t> outputPorts = this->radio->get_output_ports();

    for (size_t i = 0; i < numDdcChannels; ++i) {
        frontend_tuner_status_struct_struct &fts = this->frontend_tuner_status[currentStatus];

        fts.bandwidth = this->radio->get_output_samp_rate(i);
        fts.center_frequency = this->radio->get_rx_frequency(i);
        fts.sample_rate = this->radio->get_output_samp_rate(i);

        ++currentStatus;
    }

    std::vector<size_t> inputPorts = this->radio->get_input_ports();

    for (size_t i = 0; i < numDucChannels; ++i) {
        frontend_tuner_status_struct_struct &fts = this->frontend_tuner_status[currentStatus];

        fts.bandwidth = this->radio->get_input_samp_rate(i);
        fts.center_frequency = this->radio->get_tx_frequency(i);
        fts.sample_rate = this->radio->get_input_samp_rate(i);
        fts.tuner_type = "TX";

        ++currentStatus;
    }
}

void RFNoC_ProgrammableDevice_i::connectionAdded(const char *connectionID)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "New connection with ID: " << connectionID);

    if (this->activeDeviceID.empty()) {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "No active persona, unable to provide output");
    }

    std::map<std::string, RxObject *>::iterator it = this->allocationIDToRx.find(connectionID);

    if (it == this->allocationIDToRx.end()) {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Unable to find RX object for allocation/connection ID: " << connectionID);
        return;
    }

    bulkio::OutShortPort::ConnectionsList connections = this->dataShort_out->getConnections();

    for (size_t i = 0; i < connections.size(); ++i) {
        if (connections[i].second == connectionID) {
            CORBA::ULong providesHash = connections[i].first->_hash(1024);

            LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Found connection ID with provides hash: " << providesHash);

            BlockInfo blockInfo = this->deviceIDToGetBlockInfo[this->activeDeviceID](providesHash);

            if (not uhd::rfnoc::block_id_t::is_valid_block_id(blockInfo.blockID)) {
                LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Persona does not recognize this hash, starting a stream thread");

                size_t tunerID = getTunerMapping(connectionID);

                retrieveRxStream(tunerID);

                it->second->output.resize(10*it->second->spp);

                it->second->rxThread = new GenericThreadedComponent(boost::bind(&RFNoC_ProgrammableDevice_i::rxServiceFunction, this, tunerID));

                if (this->_started) {
                    startRxStream(tunerID);

                    it->second->rxThread->start();
                }
            } else {
                uhd::rfnoc::block_id_t blockToConnect = blockInfo.blockID;
                size_t blockPort = blockInfo.port;
                uhd::rfnoc::ddc_block_ctrl::sptr ddc = it->second->ddc;
                size_t ddcPort = it->second->ddcPort;

                this->radioChainGraph->connect(ddc->get_block_id(), ddcPort, blockToConnect, blockPort);

                it->second->connected = true;

                this->dataShort_out->pushSRI(it->second->sri);
            }

            break;
        }
    }
}

void RFNoC_ProgrammableDevice_i::connectionRemoved(const char *connectionID)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Removed connection with ID: " << connectionID);

    std::map<std::string, RxObject *>::iterator it = this->allocationIDToRx.find(connectionID);

    if (it == this->allocationIDToRx.end()) {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Unable to find RX object for allocation/connection ID: " << connectionID);
        return;
    }

    if (it->second->rxStream.get()) {
        size_t tunerID = getTunerMapping(connectionID);

        if (not it->second->rxThread->stop()) {
            LOG_WARN(RFNoC_ProgrammableDevice_i, "RX Thread had to be killed");
        }

        stopRxStream(tunerID);

        it->second->rxStream.reset();

        delete it->second->rxThread;
        it->second->rxThread = NULL;
    } else {
        it->second->connected = false;
    }
}

void RFNoC_ProgrammableDevice_i::desiredRxChannelsChanged(const unsigned char &oldValue, const unsigned char &newValue)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    if (this->allocationIDToRx.size() != 0 or this->allocationIDToTx.size() != 0) {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to change the desired number of RX channels while allocated");
        this->desiredRxChannels = oldValue;
    }
}

void RFNoC_ProgrammableDevice_i::desiredTxChannelsChanged(const unsigned char &oldValue, const unsigned char &newValue)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    if (this->allocationIDToRx.size() != 0 or this->allocationIDToTx.size() != 0) {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to change the desired number of TX channels while allocated");
        this->desiredTxChannels = oldValue;
    }
}

void RFNoC_ProgrammableDevice_i::target_deviceChanged(const target_device_struct &oldValue, const target_device_struct &newValue)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to set the target_device while running. Must be set in DCD file.");

    this->target_device = oldValue;
}

int RFNoC_ProgrammableDevice_i::rxServiceFunction(size_t streamIndex)
{
    // Perform RX, if necessary
    if (this->tunerIDToRx[streamIndex]->used) {
        // Get references to the members
        std::vector<std::complex<short> > &output = this->tunerIDToRx[streamIndex]->output;
        uhd::rx_streamer::sptr &rxStream = this->tunerIDToRx[streamIndex]->rxStream;
        BULKIO::StreamSRI &sri = this->tunerIDToRx[streamIndex]->sri;

        // Push SRI if necessary
        if (this->tunerIDToRx[streamIndex]->updateSRI) {
            LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Pushing SRI");

            this->dataShort_out->pushSRI(sri);

            this->tunerIDToRx[streamIndex]->updateSRI = false;
        }

        // Recv from the block
        uhd::rx_metadata_t md;

        LOG_TRACE(RFNoC_ProgrammableDevice_i, "Calling recv on the rx_stream");

        size_t num_rx_samps = rxStream->recv(&output.front(), output.size(), md, 3.0);

        // Check the meta data for error codes
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            LOG_ERROR(RFNoC_ProgrammableDevice_i, "Timeout while streaming");
            return NOOP;
        } else if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
            LOG_WARN(RFNoC_ProgrammableDevice_i, "Overflow while streaming");
        } else if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            LOG_WARN(RFNoC_ProgrammableDevice_i, md.strerror());
            return NOOP;
        }

        LOG_TRACE(RFNoC_ProgrammableDevice_i, "RX Thread Requested " << output.size() << " samples");
        LOG_TRACE(RFNoC_ProgrammableDevice_i, "RX Thread Received " << num_rx_samps << " samples");

        // Get the time stamps from the meta data
        BULKIO::PrecisionUTCTime rxTime;

        rxTime.twsec = md.time_spec.get_full_secs();
        rxTime.tfsec = md.time_spec.get_frac_secs();

        // Write the data to the output stream
        this->dataShort_out->pushPacket((short *) output.data(), output.size() * 2, rxTime, md.end_of_burst, sri.streamID._ptr);
    }

    return NORMAL;
}

int RFNoC_ProgrammableDevice_i::txServiceFunction(size_t streamIndex)
{
    // Perform TX, if necessary
    if (this->tunerIDToTx[streamIndex]->used) {
        // Wait on input data
        bulkio::InShortPort::DataTransferType *packet = this->dataShort_in->getPacket(bulkio::Const::BLOCKING);

        if (not packet) {
            return NOOP;
        }

        // Respond to the SRI changing
        if (packet->sriChanged) {
            LOG_DEBUG(RFNoC_ProgrammableDevice_i, "SRI Changed. Does this device care? Maybe set the DUC");
        }

        // Get references to the members
        uhd::tx_streamer::sptr &txStream = this->tunerIDToTx[streamIndex]->txStream;

        // Prepare the metadata
        uhd::tx_metadata_t md;
        std::complex<short> *block = (std::complex<short> *) packet->dataBuffer.data();
        size_t blockSize = packet->dataBuffer.size() / 2;

        LOG_TRACE(RFNoC_ProgrammableDevice_i, "TX Thread Received " << blockSize << " samples");

        if (blockSize == 0) {
            LOG_TRACE(RFNoC_ProgrammableDevice_i, "Skipping empty packet");
            delete packet;
            return NOOP;
        }

        // Get the timestamp to send to the RF-NoC block
        BULKIO::PrecisionUTCTime time = packet->T;

        md.has_time_spec = true;
        md.time_spec = uhd::time_spec_t(time.twsec, time.tfsec);

        // Send the data
        size_t num_tx_samps = txStream->send(block, blockSize, md);

        if (blockSize != 0 and num_tx_samps == 0) {
            LOG_DEBUG(RFNoC_ProgrammableDevice_i, "The TX stream is no longer valid, obtaining a new one");

            retrieveTxStream(streamIndex);
        }

        LOG_TRACE(RFNoC_ProgrammableDevice_i, "TX Thread Sent " << num_tx_samps << " samples");

        // On EOS, forward to the RF-NoC Block
        if (packet->EOS) {
            LOG_DEBUG(RFNoC_ProgrammableDevice_i, "EOS");

            md.end_of_burst = true;

            std::vector<std::complex<short> > empty;
            txStream->send(&empty.front(), empty.size(), md);
        }

        delete packet;
    }

    return NORMAL;
}

void RFNoC_ProgrammableDevice_i::setGetBlockInfoFromHash(const std::string &deviceID, getBlockInfoFromHashCallback getBlockInfoFromHashCb)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    this->deviceIDToGetBlockInfo[deviceID] = getBlockInfoFromHashCb;
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

    size_t radioChannel;
    std::map<size_t, RxObject *>::iterator rxIt = this->tunerIDToRx.find(tuner_id);
    std::map<size_t, TxObject *>::iterator txIt = this->tunerIDToTx.find(tuner_id);

    if (rxIt != this->tunerIDToRx.end()) {
        radioChannel = rxIt->second->radioChannel;
    } else if (txIt != this->tunerIDToTx.end()) {
        radioChannel = txIt->second->radioChannel;
    } else {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to enable tuner with invalid ID");
        return;
    }

    if (fts.tuner_type == "RX_DIGITIZER") {
        this->radio->set_rx_streamer(true, radioChannel);
    } else if (fts.tuner_type == "TX") {
        this->radio->set_tx_streamer(true, radioChannel);
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

    size_t radioChannel;
    std::map<size_t, RxObject *>::iterator rxIt = this->tunerIDToRx.find(tuner_id);
    std::map<size_t, TxObject *>::iterator txIt = this->tunerIDToTx.find(tuner_id);

    if (rxIt != this->tunerIDToRx.end()) {
        radioChannel = rxIt->second->radioChannel;
    } else if (txIt != this->tunerIDToTx.end()) {
        radioChannel = txIt->second->radioChannel;
    } else {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to disable tuner with invalid ID");
        return;
    }

    if (fts.tuner_type == "RX_DIGITIZER") {
        this->radio->set_rx_streamer(false, radioChannel);
    } else if (fts.tuner_type == "TX") {
        this->radio->set_tx_streamer(false, radioChannel);
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
    size_t radioChannel;
    std::map<size_t, RxObject *>::iterator rxIt = this->tunerIDToRx.find(tuner_id);
    std::map<size_t, TxObject *>::iterator txIt = this->tunerIDToTx.find(tuner_id);
    bool used;

    if (rxIt != this->tunerIDToRx.end()) {
        radioChannel = rxIt->second->radioChannel;
        used = rxIt->second->used;
    } else if (txIt != this->tunerIDToTx.end()) {
        radioChannel = txIt->second->radioChannel;
        used = txIt->second->used;
    } else {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to set tuning with invalid ID");
        return false;
    }

    // Make sure it isn't already in use
    if (used) {
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

    if (not frontend::validateRequest(125e3, 16e6, request.sample_rate)) {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Failed to set tuning: Sample rate doesn't fall between 125 kSps and 16 MSps");
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
    fts.sample_rate = request.sample_rate;

    // Map the allocation ID to the flow object
    if (request.tuner_type == "RX_DIGITIZER") {
        this->allocationIDToRx[request.allocation_id] = this->tunerIDToRx[tuner_id];

        uhd::rfnoc::ddc_block_ctrl::sptr ddc = this->tunerIDToRx[tuner_id]->ddc;
        size_t ddcPort = this->tunerIDToRx[tuner_id]->ddcPort;

        uhd::device_addr_t args;

        args["freq"] = "0.0";
        args["input_rate"] = boost::lexical_cast<std::string>(this->radio->get_output_samp_rate(radioChannel));
        args["output_rate"] = boost::lexical_cast<std::string>(fts.sample_rate);

        try {
            ddc->set_args(args, ddcPort);
        } catch(uhd::value_error &e) {
            LOG_ERROR(RFNoC_ProgrammableDevice_i, "Error while setting rates on DDC RF-NoC block: " << e.what());
            return false;
        } catch(...) {
            LOG_ERROR(RFNoC_ProgrammableDevice_i, "Unknown error occurred while setting rates on DDC RF-NoC block");
            return false;
        }

        // creates a stream id if not already created for this tuner
        std::string stream_id = getStreamId(tuner_id);

        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Got stream ID: " << stream_id);

        // enable multi-out capability for this stream/allocation/connection
        matchAllocationIdToStreamId(request.allocation_id, stream_id, "dataShort_out");

        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Matched allocation ID to stream ID: " << stream_id << " -> " << request.allocation_id);

        // Push SRI
        this->tunerIDToRx[tuner_id]->sri = create(stream_id, fts);

        // Mark this radio as used
        this->tunerIDToRx[tuner_id]->used = true;
    } else if (request.tuner_type == "TX") {
        this->allocationIDToTx[request.allocation_id] = this->tunerIDToTx[tuner_id];

        uhd::rfnoc::duc_block_ctrl::sptr duc = this->tunerIDToTx[tuner_id]->duc;
        size_t ducPort = this->tunerIDToTx[tuner_id]->ducPort;

        uhd::device_addr_t args;

        args["freq"] = boost::lexical_cast<std::string>(fts.center_frequency);
        args["input_rate"] = boost::lexical_cast<std::string>(fts.sample_rate);
        args["output_rate"] = boost::lexical_cast<std::string>(this->radio->get_input_samp_rate(radioChannel));

        try {
            duc->set_args(args, ducPort);
        } catch(uhd::value_error &e) {
            LOG_ERROR(RFNoC_ProgrammableDevice_i, "Error while setting rates on DDC RF-NoC block: " << e.what());
            return false;
        } catch(...) {
            LOG_ERROR(RFNoC_ProgrammableDevice_i, "Unknown error occurred while setting rates on DDC RF-NoC block");
            return false;
        }

        // Mark this radio as used
        this->tunerIDToTx[tuner_id]->used = true;
    }

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Allocation succeeded on: " << this->radio->get_block_id().to_string() << ":" << radioChannel);

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

    // Get the control allocation ID
    const std::string allocationId = getControlAllocationId(tuner_id);

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Found allocation ID for tuner: " << allocationId);

    // Clear the allocation ID to Rx/Tx flow objects
    std::map<std::string, RxObject *>::iterator rxIt = this->allocationIDToRx.find(allocationId);

    if (rxIt != this->allocationIDToRx.end()) {
        this->allocationIDToRx.erase(rxIt);
    }

    std::map<std::string, TxObject *>::iterator txIt = this->allocationIDToTx.find(allocationId);

    if (txIt != this->allocationIDToTx.end()) {
        this->allocationIDToTx.erase(txIt);
    }

    // Clear the objects
    if (this->tunerIDToRx.find(tuner_id) != this->tunerIDToRx.end()) {
        this->tunerIDToRx[tuner_id]->used = false;
    } else if (this->tunerIDToTx.find(tuner_id) != this->tunerIDToTx.end()) {
        this->tunerIDToTx[tuner_id]->used = false;
    }

    return true;
}

void RFNoC_ProgrammableDevice_i::clearFlows()
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Clear the map of allocation IDs to flow objects
    this->allocationIDToRx.clear();
    this->allocationIDToTx.clear();

    // Clear the map of tuner IDs to flow objects
    for (size_t i = 0; i < this->tunerIDToRx.size(); ++i) {
        if (this->tunerIDToRx[i]) {
            delete this->tunerIDToRx[i];
        }
    }

    for (size_t i = 0; i < this->tunerIDToTx.size(); ++i) {
        if (this->tunerIDToTx[i]) {
            delete this->tunerIDToTx[i];
        }
    }

    this->tunerIDToRx.clear();
    this->tunerIDToTx.clear();
}

BULKIO::StreamSRI RFNoC_ProgrammableDevice_i::create(std::string &stream_id, frontend_tuner_status_struct_struct &frontend_status, double collector_frequency) {
    BULKIO::StreamSRI sri;
    sri.hversion = 1;
    sri.xstart = 0.0;
    if ( frontend_status.sample_rate <= 0.0 )
        sri.xdelta =  1.0;
    else
        sri.xdelta = 1/frontend_status.sample_rate;
    sri.xunits = BULKIO::UNITS_TIME;
    sri.subsize = 0;
    sri.ystart = 0.0;
    sri.ydelta = 0.0;
    sri.yunits = BULKIO::UNITS_NONE;
    sri.mode = 0;
    sri.blocking=false;
    sri.streamID = stream_id.c_str();
    CORBA::Double colFreq;
    if (collector_frequency < 0)
        colFreq = frontend_status.center_frequency;
    else
        colFreq = CORBA::Double(collector_frequency);
    this->addModifyKeyword<CORBA::Double > (&sri, "COL_RF", CORBA::Double(colFreq));
    this->addModifyKeyword<CORBA::Double > (&sri, "CHAN_RF", CORBA::Double(frontend_status.center_frequency));
    this->addModifyKeyword<std::string> (&sri,"FRONTEND::RF_FLOW_ID",frontend_status.rf_flow_id);
    this->addModifyKeyword<CORBA::Double> (&sri,"FRONTEND::BANDWIDTH", CORBA::Double(frontend_status.bandwidth));
    this->addModifyKeyword<std::string> (&sri,"FRONTEND::DEVICE_ID",std::string(this->_identifier));
    return sri;
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
        LOG_DEBUG(RFNoC_ProgrammableDevice_i,"RFNoC_ProgrammableDevice_i::getStreamId - created NEW stream id: "<< frontend_tuner_status[tuner_id].stream_id);

        if (this->tunerIDToRx.find(tuner_id) != this->tunerIDToRx.end()) {
            this->tunerIDToRx[tuner_id]->updateSRI = true;
        }
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

void RFNoC_ProgrammableDevice_i::retrieveRxStream(size_t streamIndex)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Release the old stream if necessary
    if (this->tunerIDToRx[streamIndex]->rxStream.get()) {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Releasing old RX stream");
        this->tunerIDToRx[streamIndex]->rxStream.reset();
    }

    uhd::rfnoc::ddc_block_ctrl::sptr ddc = this->tunerIDToRx[streamIndex]->ddc;
    size_t ddcPort = this->tunerIDToRx[streamIndex]->ddcPort;

    // Set the stream arguments
    // Only support short complex for now
    uhd::stream_args_t stream_args("sc16", "sc16");
    uhd::device_addr_t streamer_args;

    streamer_args["block_id"] = ddc->get_block_id();

    // Get the spp from the block
    this->tunerIDToRx[streamIndex]->spp = ddc->get_args(ddcPort).cast<size_t>("spp", 1024);

    streamer_args["block_port"] = boost::lexical_cast<std::string>(ddcPort);
    streamer_args["spp"] = boost::lexical_cast<std::string>(this->tunerIDToRx[streamIndex]->spp);

    stream_args.args = streamer_args;

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Using streamer arguments: " << stream_args.args.to_string());

    // Retrieve the RX stream as specified from the device 3
    try {
        this->tunerIDToRx[streamIndex]->rxStream = this->usrp->get_rx_stream(stream_args);
    } catch(uhd::runtime_error &e) {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Failed to retrieve RX stream: " << e.what());
    } catch(...) {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Unexpected error occurred while retrieving RX stream");
    }
}

void RFNoC_ProgrammableDevice_i::retrieveTxStream(size_t streamIndex)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Release the old stream if necessary
    if (this->tunerIDToTx[streamIndex]->txStream.get()) {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Releasing old TX stream");
        this->tunerIDToTx[streamIndex]->txStream.reset();
    }

    uhd::rfnoc::duc_block_ctrl::sptr duc = this->tunerIDToTx[streamIndex]->duc;
    size_t ducPort = this->tunerIDToTx[streamIndex]->ducPort;

    // Set the stream arguments
    // Only support short complex for now
    uhd::stream_args_t stream_args("sc16", "sc16");
    uhd::device_addr_t streamer_args;

    streamer_args["block_id"] = duc->get_block_id();

    // Get the spp from the block
    this->tunerIDToTx[streamIndex]->spp = duc->get_args(ducPort).cast<size_t>("spp", 1024);

    streamer_args["block_port"] = boost::lexical_cast<std::string>(ducPort);
    streamer_args["spp"] = boost::lexical_cast<std::string>(this->tunerIDToTx[streamIndex]->spp);

    stream_args.args = streamer_args;

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Using streamer arguments: " << stream_args.args.to_string());

    // Retrieve the TX stream as specified from the device 3
    try {
        this->tunerIDToTx[streamIndex]->txStream = this->usrp->get_tx_stream(stream_args);
    } catch(uhd::runtime_error &e) {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Failed to retrieve TX stream: " << e.what());
    } catch(...) {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Unexpected error occurred while retrieving TX stream");
    }
}

void RFNoC_ProgrammableDevice_i::startRxStream(size_t streamIndex)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    if (not this->tunerIDToRx[streamIndex]->streamStarted) {
        // Start continuous streaming
        uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
        stream_cmd.num_samps = 0;
        stream_cmd.stream_now = true;
        stream_cmd.time_spec = uhd::time_spec_t();

        this->tunerIDToRx[streamIndex]->rxStream->issue_stream_cmd(stream_cmd);

        this->tunerIDToRx[streamIndex]->streamStarted = true;
    }
}

void RFNoC_ProgrammableDevice_i::stopRxStream(size_t streamIndex)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    if (this->tunerIDToRx[streamIndex]->streamStarted) {
        // Start continuous streaming
        uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);

        this->tunerIDToRx[streamIndex]->rxStream->issue_stream_cmd(stream_cmd);

        this->tunerIDToRx[streamIndex]->streamStarted = false;
    }
}
