// Class Include
#include "RFNoC_ProgrammableDevice.h"

// Local Include(s)
#include "image_loader.hpp"

// Boost Include(s)
#include <boost/filesystem.hpp>

PREPARE_LOGGING(RFNoC_ProgrammableDevice_i)

static const std::string RADIO_BLOCK_NAME = "Radio";
static const std::string DDC_BLOCK_NAME   = "DDC";
static const std::string DUC_BLOCK_NAME   = "DUC";

/*
 * Constructor(s) and/or Destructor
 */

RFNoC_ProgrammableDevice_i::RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl) :
		RFNoC_ProgrammableDevice_prog_base_type(devMgr_ior, id, lbl, sftwrPrfl),
		DEFAULT_BITFILE_PATH("/usr/share/uhd/images/usrp_e310_fpga.bit"),
		HARDWARE_ID("xc7z020clg484-1"),
		IDLE_BITFILE_PATH("/usr/share/uhd/images/usrp_e3xx_fpga_idle.bit"),
		canUnlink(true),
		DigitalTuner_in(NULL)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

RFNoC_ProgrammableDevice_i::RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev) :
		RFNoC_ProgrammableDevice_prog_base_type(devMgr_ior, id, lbl, sftwrPrfl),
		DEFAULT_BITFILE_PATH("/usr/share/uhd/images/usrp_e310_fpga.bit"),
		HARDWARE_ID("xc7z020clg484-1"),
		IDLE_BITFILE_PATH("/usr/share/uhd/images/usrp_e3xx_fpga_idle.bit"),
		canUnlink(true),
		DigitalTuner_in(NULL)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

RFNoC_ProgrammableDevice_i::RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities) :
		RFNoC_ProgrammableDevice_prog_base_type(devMgr_ior, id, lbl, sftwrPrfl, capacities),
		DEFAULT_BITFILE_PATH("/usr/share/uhd/images/usrp_e310_fpga.bit"),
		HARDWARE_ID("xc7z020clg484-1"),
		IDLE_BITFILE_PATH("/usr/share/uhd/images/usrp_e3xx_fpga_idle.bit"),
		canUnlink(true),
		DigitalTuner_in(NULL)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

RFNoC_ProgrammableDevice_i::RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev) :
		RFNoC_ProgrammableDevice_prog_base_type(devMgr_ior, id, lbl, sftwrPrfl, capacities, compDev),
		DEFAULT_BITFILE_PATH("/usr/share/uhd/images/usrp_e310_fpga.bit"),
		HARDWARE_ID("xc7z020clg484-1"),
		IDLE_BITFILE_PATH("/usr/share/uhd/images/usrp_e3xx_fpga_idle.bit"),
		canUnlink(true),
		DigitalTuner_in(NULL)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

RFNoC_ProgrammableDevice_i::~RFNoC_ProgrammableDevice_i()
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    delete this->DigitalTuner_in;
}

/*
 * Public Method(s)
 */

bool RFNoC_ProgrammableDevice_i::connectRadioRX(RFNoC_RH::PortHashType portHash, const RFNoC_RH::BlockDescriptor &blockDescriptor)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    if (not this->graph)
    {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Unable to connect radio without graph");

        return false;
    }

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Checking output port for hash " << portHash << " to connect programmable graph to " << blockDescriptor.blockId.to_string());

    bulkio::OutShortPort::ConnectionsList connections = this->dataShort_out->getConnections();

    for (size_t i = 0; i < connections.size(); ++i)
    {
        CORBA::ULong providesHash = connections[i].first->_hash(RFNoC_RH::HASH_SIZE);

        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Checking hash " << providesHash);

        if (providesHash == portHash)
        {
            LOG_INFO(RFNoC_ProgrammableDevice_i, "Found correct connection, retrieving DDC information");

            std::string connectionId = connections[i].second;

            allocationIdToRxMap::iterator it = this->allocationIdToRx.find(connectionId);

            if (it == this->allocationIdToRx.end())
            {
                LOG_WARN(RFNoC_ProgrammableDevice_i, "Unable to find RX object for allocation/connection ID: " << connectionId);

                continue;
            }
            else if (it->second->connected)
            {
                LOG_DEBUG(RFNoC_ProgrammableDevice_i, "RX object already connected for allocation/connection ID: " << connectionId);

                return true;
            }

            uhd::rfnoc::ddc_block_ctrl::sptr ddc = it->second->ddc;
            size_t ddcPort = it->second->ddcPort;

            this->graph->connect(ddc->unique_id(), ddcPort, blockDescriptor.blockId, blockDescriptor.port);

            it->second->connected = true;

            it->second->downstreamBlock = this->usrp->get_block_ctrl(blockDescriptor.blockId);
            it->second->downstreamBlockPort = blockDescriptor.port;

            return true;
        }
    }

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "No connection possible");

    return false;
}

bool RFNoC_ProgrammableDevice_i::connectRadioTX(const std::string &allocationId, const RFNoC_RH::BlockDescriptor &blockDescriptor)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    if (not this->graph)
    {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Unable to connect radio without graph");

        return false;
    }

    allocationIdToTxMap::iterator it = this->allocationIdToTx.find(allocationId);

    if (it == this->allocationIdToTx.end())
    {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to connect to DUC with unknown allocation ID: " << allocationId);

        return false;
    }

    uhd::rfnoc::duc_block_ctrl::sptr duc = it->second->duc;
    size_t ducPort = it->second->ducPort;

    this->graph->connect(blockDescriptor.blockId, blockDescriptor.port, duc->unique_id(), ducPort);

    return true;
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

    for (size_t i = 0; i < parameters.length(); ++i)
    {
        std::string id = parameters[i].id._ptr;

        if (id == "DEVICE_ID")
        {
            deviceIdentifier = ossie::any_to_string(parameters[i].value);
            break;
        }
    }

    this->pidToDeviceId[pid] = deviceIdentifier;

    return pid;
}

void RFNoC_ProgrammableDevice_i::releaseObject() throw (CF::LifeCycle::ReleaseError, CORBA::SystemException)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Load the idle bitfile
    loadBitfile(this->IDLE_BITFILE_PATH);

    // Remove the symbolic link, if necessary
    if (this->canUnlink)
    {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Removing symbolic link from requested bitfile path");

        if (not boost::filesystem::remove(this->DEFAULT_BITFILE_PATH))
        {
            LOG_ERROR(RFNoC_ProgrammableDevice_i, "A problem occurred while unlinking the default bitfile symbolic link");
        }
    }

    RFNoC_ProgrammableDevice_prog_base_type::releaseObject();
}

void RFNoC_ProgrammableDevice_i::terminate (CF::ExecutableDevice::ProcessID_Type processId)
                    throw ( CF::Device::InvalidState, CF::ExecutableDevice::InvalidProcess, CORBA::SystemException)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    if (this->pidToDeviceId.find(processId) == this->pidToDeviceId.end())
    {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to terminate a process with an ID not tracked by this Device: " << processId);
        throw CF::ExecutableDevice::InvalidProcess();
    }

    // Get the device identifier associated with this PID
    std::string deviceID = this->pidToDeviceId[processId];

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Terminating device: " << deviceID);

    // Unmap the device identifier from the persona
    this->deviceIdToPersona.erase(deviceID);

    // Unmap the device identifier from the HW Load Status
    this->deviceIdToHwStatus.erase(deviceID);

    // Unmap the PID from the device identifier
    this->pidToDeviceId.erase(processId);

    // Adjust the HW load statuses property
    this->hw_load_statuses.clear();

    for (deviceHwStatusMap::iterator it = this->deviceIdToHwStatus.begin(); it != this->deviceIdToHwStatus.end(); ++it)
    {
        this->hw_load_statuses.push_back(it->second);
    }

    // Call the parent terminate
    RFNoC_ProgrammableDevice_prog_base_type::terminate(processId);
}

/*
 * Public RFNoC_Programmable Method(s)
 */

bool RFNoC_ProgrammableDevice_i::connectBlocks(const RFNoC_RH::BlockDescriptor &outputDescriptor, const RFNoC_RH::BlockDescriptor &inputDescriptor)
{
	LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

	if (not this->graph)
	{
		LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to connect blocks without RF-NoC graph");

		return false;
	}

	try
	{
		this->graph->connect(outputDescriptor.blockId,
					   	     outputDescriptor.port,
							 inputDescriptor.blockId,
							 inputDescriptor.port);
	}
	catch(...)
	{
		LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to connect blocks");
		return false;
	}

	return true;
}

Resource_impl* RFNoC_ProgrammableDevice_i::generateResource(int argc, char* argv[], RFNoC_RH::RFNoC_Component_Constructor_Ptr fnptr, const char* libraryName)
{
	LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

/*
 * This method should return an RF-NoC block to the caller. If the port was unset by the
 * caller, the port used will be placed into blockDescriptor.
 */
uhd::rfnoc::block_ctrl_base::sptr RFNoC_ProgrammableDevice_i::getBlock(RFNoC_RH::BlockDescriptor &blockDescriptor)
{
	LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

	uhd::rfnoc::block_ctrl_base::sptr block;

	if (not this->usrp)
	{
		LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to get block from uninitialized USRP");
		return block;
	}

	uhd::rfnoc::block_id_t blockId = blockDescriptor.blockId;

	try
	{
		block = this->usrp->get_block_ctrl(blockId);
	}
	catch(...)
	{
		LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to get block with ID: " << blockId.get());
	}

	// TODO: Mark block and port as in use

	return block;
}

/*
 * This method should return the block descriptor corresponding to the
 * given port hash.
 */
RFNoC_RH::BlockDescriptor RFNoC_ProgrammableDevice_i::getBlockDescriptorFromHash(RFNoC_RH::PortHashType portHash)
{
	LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

void RFNoC_ProgrammableDevice_i::incomingConnectionAdded(const std::string &resourceId,
									 	 	 	 	 	 const std::string &streamId,
														 RFNoC_RH::PortHashType portHash)
{
	LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

	if (not this->resourceManager)
	{
		LOG_WARN(RFNoC_ProgrammableDevice_i, "New connection added but no RF-NoC resource manager available");

		return;
	}

	IncomingConnection incomingConnection;

	incomingConnection.portHash = portHash;
	incomingConnection.streamId = streamId;
	incomingConnection.resourceId = resourceId;

	this->resourceManager->registerIncomingConnection(incomingConnection);
}

void RFNoC_ProgrammableDevice_i::incomingConnectionRemoved(const std::string &resourceId,
									   	   	   	   	   	   const std::string &streamId,
														   RFNoC_RH::PortHashType portHash)
{
	LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

	if (not this->resourceManager)
	{
		LOG_WARN(RFNoC_ProgrammableDevice_i, "Connection removed but no RF-NoC resource manager available");

		return;
	}
}

void RFNoC_ProgrammableDevice_i::outgoingConnectionAdded(const std::string &resourceId,
									 	 	 	 	 	 const std::string &connectionId,
														 RFNoC_RH::PortHashType portHash)
{
	LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

void RFNoC_ProgrammableDevice_i::outgoingConnectionRemoved(const std::string &resourceId,
									   	   	   	   	   	   const std::string &connectionId,
														   RFNoC_RH::PortHashType portHash)
{
	LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}


void RFNoC_ProgrammableDevice_i::setPersonaMapping(const std::string& deviceId, RFNoC_RH::RFNoC_Persona *persona)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    std::map<std::string, RFNoC_RH::RFNoC_Persona *>::iterator it = this->deviceIdToPersona.find(deviceId);

    if (it != this->deviceIdToPersona.end())
    {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to set persona mapping for persona which is already tracked: " << deviceId);

        return;
    }

    this->deviceIdToPersona[deviceId] = persona;
}

void RFNoC_ProgrammableDevice_i::setRxStreamDescriptor(const std::string &resourceId, const RFNoC_RH::StreamDescriptor &streamDescriptor)
{
	LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

void RFNoC_ProgrammableDevice_i::setTxStreamDescriptor(const std::string &resourceId, const RFNoC_RH::StreamDescriptor &streamDescriptor)
{
	LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

/*
 * Protected Method(s)
 */

void RFNoC_ProgrammableDevice_i::constructor()
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Set up the digital tuner in port
    this->DigitalTuner_in = new frontend::InDigitalTunerPort("DigitalTuner_in", this);
    addPort("DigitalTuner_in", this->DigitalTuner_in);

    // Set the load requests and statuses pointers to the properties
    setHwLoadRequestsPtr(&hw_load_requests);
    setHwLoadStatusesPtr(&hw_load_statuses);

    // There is only one FPGA available
    this->hw_load_statuses.resize(1);
    this->hw_load_statuses[0].hardware_id = this->HARDWARE_ID;
    this->hw_load_statuses[0].state = 0;

    // Set the usrp address
    this->usrpAddress["no_reload_fpga"] = true;

    if (not this->target_device.name.empty())
    {
        this->usrpAddress["name"] = this->target_device.name;
    }

    if (not this->target_device.serial.empty())
    {
        this->usrpAddress["serial"] = this->target_device.serial;
    }

    if (not this->target_device.type.empty())
    {
        this->usrpAddress["type"] = this->target_device.type;
    }

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Trying target_device: " << this->usrpAddress.to_pp_string());

    // Check if the default image path exists. If so, then only personas with a
    // bitfile path that matches the default can be loaded
    if (boost::filesystem::exists(this->DEFAULT_BITFILE_PATH))
    {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "\n*********************************************************************************************************************************************************");
        LOG_INFO(RFNoC_ProgrammableDevice_i, "Default bitfile path (" << this->DEFAULT_BITFILE_PATH << ") exists");
        this->canUnlink = false;

        if (boost::filesystem::is_symlink(this->DEFAULT_BITFILE_PATH))
        {
            LOG_WARN(RFNoC_ProgrammableDevice_i, "Default bitfile is a symbolic link. If this is unintentional, please release this Device, unlink the file, and restart the Device");
        }
        else
        {
            LOG_WARN(RFNoC_ProgrammableDevice_i, "Due to the presence of this file, only a Persona with a matching FPGA bitfile path can be launched. For proper behavior, please consult the documentation");
        }

        LOG_INFO(RFNoC_ProgrammableDevice_i, "\n*********************************************************************************************************************************************************");
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

/*
 * Protected RFNoC_ProgrammableDevice_prog_base_type Method(s)
 */

Device_impl* RFNoC_ProgrammableDevice_i::generatePersona(int argc, char* argv[], ConstructorPtr personaEntryPoint, const char* libName)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Generate the Persona Device
    Device_impl *persona = personaEntryPoint(argc, argv, this);

    // Something went wrong
    if (not persona)
    {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Failed to generate Persona Device. Unable to instantiate Persona Device from library: " << libName);
        return NULL;
    }

    return persona;
}

bool RFNoC_ProgrammableDevice_i::hwLoadRequestIsValid(const HwLoadRequestStruct& hwLoadRequestStruct)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    if (hwLoadRequestStruct.hardware_id != this->HARDWARE_ID)
    {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to validate hardware load request. Mismatched hardware IDs.");
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Expected: " << this->HARDWARE_ID << ", Received: " << hwLoadRequestStruct.hardware_id);

        return false;
    }

    if (not boost::filesystem::exists(hwLoadRequestStruct.load_filepath))
    {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to validate hardware load request. Load file path is invalid.");
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Invalid file path: " << hwLoadRequestStruct.load_filepath);

        return false;
    }

    if (not this->canUnlink)
    {
        if (hwLoadRequestStruct.load_filepath != this->DEFAULT_BITFILE_PATH)
        {
            LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to validate hardware load request. The default bitfile exists and the requested bitfile does not match.");
            LOG_WARN(RFNoC_ProgrammableDevice_i, "Default bitfile: " << this->DEFAULT_BITFILE_PATH);
            LOG_WARN(RFNoC_ProgrammableDevice_i, "Received bitfile: " << hwLoadRequestStruct.load_filepath);

            return false;
        }
    }

    return true;
}

bool RFNoC_ProgrammableDevice_i::loadHardware(HwLoadStatusStruct& requestStatus) 
{
    // The hardware may be physically loaded at this point
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Create a symbolic link from the default file path to the requested file
    // path, if necessary
    if (this->canUnlink)
    {
        if (boost::filesystem::exists(this->DEFAULT_BITFILE_PATH))
        {
            LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Default bitfile path exists, unlinking");

            if (not boost::filesystem::remove(this->DEFAULT_BITFILE_PATH))
            {
                LOG_ERROR(RFNoC_ProgrammableDevice_i, "A problem occurred while unlinking the default bitfile symbolic link");

                return false;
            }
        }

        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Creating symbolic link to requested bitfile path");

        boost::system::error_code ec;
        boost::filesystem::create_symlink(requestStatus.load_filepath, this->DEFAULT_BITFILE_PATH, ec);

        if (ec != boost::system::errc::success)
        {
            LOG_ERROR(RFNoC_ProgrammableDevice_i, "A problem occurred while linking the requested bitfile path");
            return false;
        }
    }

    // Load the requested bitfile
    loadBitfile(this->DEFAULT_BITFILE_PATH);

    // Allow some time for setup
    boost::this_thread::sleep(boost::posix_time::seconds(1.0));

    // Attempt to get a reference to the specified device
    try
    {
        this->usrp = uhd::device3::make(this->usrpAddress);
    }
    catch(uhd::key_error &e)
    {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Unable to find a suitable USRP Device 3.");
        LOG_ERROR(RFNoC_ProgrammableDevice_i, e.what());
        return false;
    }
    catch(std::exception &e)
    {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "An error occurred attempting to get a reference to the USRP device.");
        LOG_ERROR(RFNoC_ProgrammableDevice_i, e.what());
        return false;
    }

    // Gather all blocks
    gatherBlocks();

    // Configure FEI capabilities and connect DUC -> Radio -> DDC
    initializeRadioChain();

    // Instantiate the RF-NoC Resource Manager
    this->resourceManager = boost::make_shared<RFNoC_ResourceManager>(this, this);

    // Set the active device ID
    this->activeDeviceId = requestStatus.requester_id;

    return true;
}

void RFNoC_ProgrammableDevice_i::unloadHardware(const HwLoadStatusStruct& requestStatus) 
{
    // The hardware may be physically unloaded at this point
    LOG_INFO(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Release the RF-NoC Resource Manager
    this->resourceManager.reset();

    // Clear the USRP device
    if (this->usrp)
    {
        // This throws exceptions occasionally
        try
        {
            this->usrp->clear();
        }
        catch(...)
        {

        }
    }

    // Reset the block pointers
    this->blocks.clear();
    this->ddcs.clear();
    this->ducs.clear();
    this->radio.reset();

    // Reset the USRP pointer
    this->usrp.reset();

    // Clear the graph
    this->graph.reset();

    // Clear the flows
    clearFlows();

    // Clear the frontend_tuner_status
    setNumChannels(0);

    // Load the idle bitfile
    loadBitfile(this->IDLE_BITFILE_PATH);

    // Remove the symbolic link, if necessary
    if (this->canUnlink)
    {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Removing symbolic link from requested bitfile path");

        if (not boost::filesystem::remove(this->DEFAULT_BITFILE_PATH))
        {
            LOG_ERROR(RFNoC_ProgrammableDevice_i, "A problem occurred while unlinking the default bitfile symbolic link");
        }
    }

    // Clear the active device ID
    this->activeDeviceId.clear();
}

/*
 * Protected frontend::digital_tuner_delegation Method(s)
 */

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

    tunerIdToRxMap::iterator rxIt = this->tunerIdToRx.find(tuner_id);
    tunerIdToTxMap::iterator txIt = this->tunerIdToTx.find(tuner_id);

    if (rxIt != this->tunerIdToRx.end() && fts.tuner_type == "RX_DIGITIZER")
    {
        rxIt->second->ddc->set_rx_streamer(true, rxIt->second->ddcPort);
    }
    else if (txIt != this->tunerIdToTx.end() && fts.tuner_type == "TX")
    {
        txIt->second->duc->set_tx_streamer(true, txIt->second->ducPort);
    }
    else
    {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to disable tuner with invalid ID");

        return;
    }

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Enabled tuner " << tuner_id);

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

    tunerIdToRxMap::iterator rxIt = this->tunerIdToRx.find(tuner_id);
    tunerIdToTxMap::iterator txIt = this->tunerIdToTx.find(tuner_id);

    if (rxIt != this->tunerIdToRx.end() && fts.tuner_type == "RX_DIGITIZER")
    {
        rxIt->second->ddc->set_rx_streamer(false, rxIt->second->ddcPort);
    }
    else if (txIt != this->tunerIdToTx.end() && fts.tuner_type == "TX")
    {
        txIt->second->duc->set_tx_streamer(false, txIt->second->ducPort);
    }
    else
    {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to disable tuner with invalid ID");

        return;
    }

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Disabled tuner " << tuner_id);

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
    tunerIdToRxMap::iterator rxIt = this->tunerIdToRx.find(tuner_id);
    tunerIdToTxMap::iterator txIt = this->tunerIdToTx.find(tuner_id);
    bool used;

    if (rxIt != this->tunerIdToRx.end())
    {
        radioChannel = rxIt->second->radioChannel;
        used = rxIt->second->used;
    }
    else if (txIt != this->tunerIdToTx.end())
    {
        radioChannel = txIt->second->radioChannel;
        used = txIt->second->used;
    }
    else {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to set tuning with invalid ID");

        return false;
    }

    // Make sure it isn't already in use
    if (used)
    {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Failed to set tuning: Requested tuner already in use");

        return false;
    }

    // Attempt to set the radio with the requested values
    if (not frontend::validateRequest(16e6, 16e6, request.bandwidth))
    {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Failed to set tuning: Bandwidth doesn't match 16 MHz");

        return false;
    }

    if (not frontend::validateRequest(70e6, 6000e6, request.center_frequency))
    {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Failed to set tuning: Center frequency not between 70 MHz and 6 GHz");

        return false;
    }

    if (not frontend::validateRequest(125e3, 16e6, request.sample_rate))
    {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Failed to set tuning: Sample rate doesn't fall between 125 kSps and 16 MSps");

        return false;
    }

    // Set the center frequency
    double actualCF;

    if (request.tuner_type == "RX_DIGITIZER")
    {
        bool succeeded = false;

        try
        {
            actualCF = this->radio->set_rx_frequency(request.center_frequency, radioChannel);

            LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Requested CF: " << request.center_frequency << ", Actual CF: " << actualCF);

            succeeded = true;
        }
        catch(...)
        {

        }

        if (not succeeded)
        {
            LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to set tuning: Unable to set center frequency");

            return false;
        }
    }
    else if (request.tuner_type == "TX")
    {
        bool succeeded = false;

        try
        {
            actualCF = this->radio->set_tx_frequency(request.center_frequency, radioChannel);

            LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Requested CF: " << request.center_frequency << ", Actual CF: " << actualCF);

            succeeded = true;
        }
        catch(...)
        {

        }

        if (not succeeded)
        {
            LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to set tuning: Unable to set center frequency");

            return false;
        }
    }
    else
    {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to set tuning: Invalid tuner type");

        return false;
    }

    // Set the frontend tuner status
    fts.bandwidth = 16e6;
    fts.center_frequency = actualCF;

    // Set the sample rate
    double sampleRate;

    // Map the allocation ID to the flow object
    if (request.tuner_type == "RX_DIGITIZER")
    {
        this->allocationIdToRx[request.allocation_id] = this->tunerIdToRx[tuner_id];

        uhd::rfnoc::ddc_block_ctrl::sptr ddc = this->tunerIdToRx[tuner_id]->ddc;
        size_t ddcPort = this->tunerIdToRx[tuner_id]->ddcPort;

        uhd::device_addr_t args;

        args["freq"] = "0.0";
        args["input_rate"] = boost::lexical_cast<std::string>(this->radio->get_output_samp_rate(radioChannel));
        args["output_rate"] = boost::lexical_cast<std::string>(request.sample_rate);

        try
        {
            ddc->set_args(args, ddcPort);
        }
        catch(uhd::value_error &e)
        {
            LOG_ERROR(RFNoC_ProgrammableDevice_i, "Error while setting rates on DDC RF-NoC block: " << e.what());

            return false;
        }
        catch(...)
        {
            LOG_ERROR(RFNoC_ProgrammableDevice_i, "Unknown error occurred while setting rates on DDC RF-NoC block");

            return false;
        }

        // Get the actual output rate of the DDC
        //sampleRate = ddc->get_output_samp_rate(ddcPort);

        try
        {
            std::string sampleRateString;

            sampleRateString = ddc->get_arg("output_rate", ddcPort);

            sampleRate = boost::lexical_cast<double>(sampleRateString);
        }
        catch(...)
        {
            LOG_ERROR(RFNoC_ProgrammableDevice_i, "Error occurred while getting output rate on DDC RF-NoC block");

            return false;
        }

        // creates a stream id if not already created for this tuner
        std::string stream_id = getStreamId(tuner_id);

        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Got stream ID: " << stream_id);

        // enable multi-out capability for this stream/allocation/connection
        matchAllocationIdToStreamId(request.allocation_id, stream_id, "dataShort_out");

        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Matched allocation ID to stream ID: " << stream_id << " -> " << request.allocation_id);

        // Push SRI
        this->tunerIdToRx[tuner_id]->sri = create(stream_id, fts);
        this->tunerIdToRx[tuner_id]->sri.mode = 1;
        this->tunerIdToRx[tuner_id]->sri.xdelta = 1.0 / sampleRate;

        // Mark this radio as used
        this->tunerIdToRx[tuner_id]->used = true;
    }
    else if (request.tuner_type == "TX")
    {
        this->allocationIdToTx[request.allocation_id] = this->tunerIdToTx[tuner_id];

        uhd::rfnoc::duc_block_ctrl::sptr duc = this->tunerIdToTx[tuner_id]->duc;
        size_t ducPort = this->tunerIdToTx[tuner_id]->ducPort;

        uhd::device_addr_t args;

        args["freq"] = boost::lexical_cast<std::string>(fts.center_frequency);
        args["input_rate"] = boost::lexical_cast<std::string>(request.sample_rate);
        args["output_rate"] = boost::lexical_cast<std::string>(this->radio->get_input_samp_rate(radioChannel));

        try
        {
            duc->set_args(args, ducPort);
        }
        catch(uhd::value_error &e)
        {
            LOG_ERROR(RFNoC_ProgrammableDevice_i, "Error while setting rates on DDC RF-NoC block: " << e.what());

            return false;
        }
        catch(...)
        {
            LOG_ERROR(RFNoC_ProgrammableDevice_i, "Unknown error occurred while setting rates on DDC RF-NoC block");

            return false;
        }

        // Get the actual input rate of the DUC
        sampleRate = duc->get_input_samp_rate(ducPort);

        // Mark this radio as used
        this->tunerIdToTx[tuner_id]->used = true;
    }

    // Set the sample rate on the fts object
    fts.sample_rate = sampleRate;

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Allocation succeeded on: " << this->radio->unique_id() << ":" << radioChannel);

    return true;
}

bool RFNoC_ProgrammableDevice_i::deviceDeleteTuning(frontend_tuner_status_struct_struct &fts,
													size_t tuner_id)
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
    allocationIdToRxMap::iterator rxIt = this->allocationIdToRx.find(allocationId);

    if (rxIt != this->allocationIdToRx.end())
    {
        this->allocationIdToRx.erase(rxIt);
    }

    allocationIdToTxMap::iterator txIt = this->allocationIdToTx.find(allocationId);

    if (txIt != this->allocationIdToTx.end())
    {
        this->allocationIdToTx.erase(txIt);
    }

    // Clear the objects
    if (this->tunerIdToRx.find(tuner_id) != this->tunerIdToRx.end())
    {
        std::vector<short> empty;
        BULKIO::PrecisionUTCTime T = bulkio::time::utils::now();

        this->dataShort_out->pushPacket(empty, T, true, fts.stream_id);

        fts.stream_id.clear();
        removeAllocationIdRouting(tuner_id);
        this->tunerIdToRx[tuner_id]->used = false;
    }
    else if (this->tunerIdToTx.find(tuner_id) != this->tunerIdToTx.end())
    {
        this->tunerIdToTx[tuner_id]->used = false;
    }

    return true;
}

BULKIO::StreamSRI RFNoC_ProgrammableDevice_i::create(std::string &stream_id, frontend_tuner_status_struct_struct &frontend_status, double collector_frequency)
{
    BULKIO::StreamSRI sri;
    CORBA::Double colFreq;

    sri.hversion = 1;
    sri.xstart = 0.0;

    if ( frontend_status.sample_rate <= 0.0 )
    {
        sri.xdelta =  1.0;
    }
    else
    {
        sri.xdelta = 1/frontend_status.sample_rate;
    }

    sri.xunits = BULKIO::UNITS_TIME;
    sri.subsize = 0;
    sri.ystart = 0.0;
    sri.ydelta = 0.0;
    sri.yunits = BULKIO::UNITS_NONE;
    sri.mode = 0;
    sri.blocking=false;
    sri.streamID = stream_id.c_str();

    if (collector_frequency < 0)
    {
        colFreq = frontend_status.center_frequency;
    }
    else
    {
        colFreq = CORBA::Double(collector_frequency);
    }

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

        if (this->tunerIdToRx.find(tuner_id) != this->tunerIdToRx.end()) {
            this->tunerIdToRx[tuner_id]->updateSRI = true;
        }
    } else {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i,"RFNoC_ProgrammableDevice_i::getStreamId - returning EXISTING stream id: "<< frontend_tuner_status[tuner_id].stream_id);
    }
    return frontend_tuner_status[tuner_id].stream_id;
}

/*
 * Private Method(s)
 */

void RFNoC_ProgrammableDevice_i::clearFlows()
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Clear the map of allocation IDs to flow objects
    this->allocationIdToRx.clear();
    this->allocationIdToTx.clear();

    // Clear the map of tuner IDs to flow objects
    this->tunerIdToRx.clear();
    this->tunerIdToTx.clear();
}

void RFNoC_ProgrammableDevice_i::connectionAdded(const char *connectionID)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "New connection with ID: " << connectionID);

    if (this->activeDeviceId.empty())
    {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "No active persona, unable to provide output");
    }

    allocationIdToRxMap::iterator it = this->allocationIdToRx.find(connectionID);

    if (it == this->allocationIdToRx.end())
    {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Unable to find RX object for allocation/connection ID: " << connectionID);

        return;
    }

    bulkio::OutShortPort::ConnectionsList connections = this->dataShort_out->getConnections();

    for (size_t i = 0; i < connections.size(); ++i)
    {
        if (connections[i].second == connectionID)
        {
            RFNoC_RH::PortHashType providesHash = connections[i].first->_hash(RFNoC_RH::HASH_SIZE);

            LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Found connection ID with provides hash: " << providesHash);

            RFNoC_RH::BlockDescriptor blockDescriptor = this->deviceIdToPersona[this->activeDeviceId]->getBlockDescriptorFromHash(providesHash);

            if (not uhd::rfnoc::block_id_t::is_valid_block_id(blockDescriptor.blockId))
            {
                LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Persona does not recognize this hash, starting a stream thread");

                size_t tunerID = getTunerMapping(connectionID);

                retrieveRxStream(tunerID);

                it->second->output.resize(10*it->second->spp);

                it->second->rxThread = boost::make_shared<RFNoC_RH::GenericThreadedComponent>(boost::bind(&RFNoC_ProgrammableDevice_i::rxServiceFunction, this, tunerID));

                if (this->_started)
                {
                    startRxStream(tunerID);

                    it->second->rxThread->start();
                }
            }
            else
            {
                uhd::rfnoc::block_id_t blockToConnect = blockDescriptor.blockId;
                size_t blockPort = blockDescriptor.port;
                uhd::rfnoc::ddc_block_ctrl::sptr ddc = it->second->ddc;
                size_t ddcPort = it->second->ddcPort;

                this->graph->connect(ddc->unique_id(), ddcPort, blockToConnect, blockPort);

                it->second->connected = true;

                it->second->downstreamBlock = this->usrp->get_block_ctrl(blockToConnect);
                it->second->downstreamBlockPort = blockPort;

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

    allocationIdToRxMap::iterator it = this->allocationIdToRx.find(connectionID);

    if (it == this->allocationIdToRx.end())
    {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Unable to find RX object for allocation/connection ID: " << connectionID);

        return;
    }

    if (it->second->rxStream)
    {
        size_t tunerID = getTunerMapping(connectionID);

        if (not it->second->rxThread->stop())
        {
            LOG_WARN(RFNoC_ProgrammableDevice_i, "RX Thread had to be killed");
        }

        stopRxStream(tunerID);

        it->second->rxStream.reset();

        it->second->rxThread.reset();
    }
    else
    {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Disconnecting DDC from downstream block in fabric");

        try
        {
            it->second->ddc->disconnect_output_port(it->second->ddcPort);
        }
        catch(...)
        {
            LOG_WARN(RFNoC_ProgrammableDevice_i, "Exception occurred while disconnecting " << it->second->ddc->unique_id());
        }

        if (it->second->downstreamBlock.get())
        {
            LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Disconnecting downstream block from DDC in fabric");

            try
            {
                it->second->downstreamBlock->disconnect_input_port(it->second->downstreamBlockPort);
            }
            catch(...)
            {
                LOG_WARN(RFNoC_ProgrammableDevice_i, "Exception occurred while disconnecting " << it->second->downstreamBlock->unique_id());
            }

            it->second->downstreamBlock.reset();
        }
        else
        {
            LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Downstream block was not connected");
        }

        it->second->connected = false;
        it->second->downstreamBlock.reset();

        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Successfully disconnected");
    }
}

void RFNoC_ProgrammableDevice_i::desiredRxChannelsChanged(const unsigned char &oldValue, const unsigned char &newValue)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    if (this->allocationIdToRx.size() != 0 or this->allocationIdToTx.size() != 0)
    {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to change the desired number of RX channels while allocated");

        this->desiredRxChannels = oldValue;
    }
}

void RFNoC_ProgrammableDevice_i::desiredTxChannelsChanged(const unsigned char &oldValue, const unsigned char &newValue)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    if (this->allocationIdToRx.size() != 0 or this->allocationIdToTx.size() != 0)
    {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to change the desired number of TX channels while allocated");

        this->desiredTxChannels = oldValue;
    }
}

void RFNoC_ProgrammableDevice_i::gatherBlocks()
{
	LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

	this->blocks.clear();
	this->ddcs.clear();
	this->ducs.clear();
	this->radio.reset();

	if (not this->usrp)
	{
		LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to gather blocks without initialized USRP");
		return;
	}

	std::vector<uhd::rfnoc::block_id_t> radioIds = this->usrp->find_blocks(RADIO_BLOCK_NAME);
	std::vector<uhd::rfnoc::block_id_t> ddcIds   = this->usrp->find_blocks(DDC_BLOCK_NAME);
	std::vector<uhd::rfnoc::block_id_t> ducIds   = this->usrp->find_blocks(DUC_BLOCK_NAME);
	std::vector<uhd::rfnoc::block_id_t> blockIds = radioIds;
	blockIds.insert(blockIds.end(), ddcIds.begin(), ddcIds.end());
	blockIds.insert(blockIds.end(), ducIds.begin(), ducIds.end());

	for (size_t i = 0; i < blockIds.size(); ++i)
	{
		LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Getting block with block ID: " << blockIds[i].get());

		if (blockIds[i].match(RADIO_BLOCK_NAME))
		{
			try
			{
				this->radio = this->usrp->get_block_ctrl<uhd::rfnoc::radio_ctrl>(blockIds[i]);
			}
			catch(...)
			{
				LOG_WARN(RFNoC_ProgrammableDevice_i, "An error occurred while retrieving block");
			}
		}
		else if (blockIds[i].match(DDC_BLOCK_NAME))
		{
			uhd::rfnoc::ddc_block_ctrl::sptr ddc;

			try
			{
				ddc = this->usrp->get_block_ctrl<uhd::rfnoc::ddc_block_ctrl>(blockIds[i]);
			}
			catch(...)
			{
				LOG_WARN(RFNoC_ProgrammableDevice_i, "An error occurred while retrieving block");
				continue;
			}

			this->ddcs.push_back(ddc);
		}
		else if (blockIds[i].match(DUC_BLOCK_NAME))
		{
			uhd::rfnoc::duc_block_ctrl::sptr duc;

			try
			{
				duc = this->usrp->get_block_ctrl<uhd::rfnoc::duc_block_ctrl>(blockIds[i]);
			}
			catch(...)
			{
				LOG_WARN(RFNoC_ProgrammableDevice_i, "An error occurred while retrieving block");
				continue;
			}

			this->ducs.push_back(duc);
		}
		else
		{
			uhd::rfnoc::block_ctrl_base::sptr block;

			try
			{
				block = this->usrp->get_block_ctrl(blockIds[i]);
			}
			// TODO: Implement more specific error handling
			catch(...)
			{
				LOG_WARN(RFNoC_ProgrammableDevice_i, "An error occurred while retrieving block");
				continue;
			}

			std::pair<uhd::rfnoc::block_id_t, uhd::rfnoc::block_ctrl_base::sptr> entry = std::make_pair(blockIds[i], block);

			std::pair<blockIdToBlockMap::iterator, bool> entryResult = this->blocks.insert(entry);

			if (not entryResult.second)
			{
				LOG_WARN(RFNoC_ProgrammableDevice_i, "An error occurred while inserting block into map");
			}
		}
	}
}

void RFNoC_ProgrammableDevice_i::initializeRadioChain()
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Create the RF-NoC graph
    this->graph = this->usrp->create_graph("programmableGraph");

    if (not this->graph)
    {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Failed to get the programmable graph");

        return;
    }

    // Gather DDC Information
    size_t numDdcChannels = 0;

    for (size_t i = 0; i < this->ddcs.size(); ++i)
    {
        numDdcChannels += this->ddcs[i]->get_input_ports().size();
    }

    // Gather DUC Information
    size_t numDucChannels = 0;

    for (size_t i = 0; i < this->ducs.size(); ++i)
    {
        numDucChannels += this->ducs[i]->get_input_ports().size();
    }

    // Check the properties
    if (this->desiredRxChannels > numDdcChannels)
    {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Desired number of RX channels greater than available DDC channels.");

        this->desiredRxChannels = numDdcChannels;
    }
    else
    {
        numDdcChannels = this->desiredRxChannels;
    }

    if (this->desiredTxChannels > numDucChannels)
    {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Desired number of TX channels greater than available DUC channels.");

        this->desiredTxChannels = numDucChannels;
    }
    else
    {
        numDucChannels = this->desiredTxChannels;
    }

    // Instantiate the flow objects
    size_t tunerID = 0;

    for (size_t i = 0; i < numDdcChannels; ++i)
    {
        this->tunerIdToRx[tunerID] = boost::make_shared<RxObject>();

        this->tunerIdToRx[tunerID]->connected = false;
        this->tunerIdToRx[tunerID]->streamStarted = false;
        this->tunerIdToRx[tunerID]->updateSRI = false;
        this->tunerIdToRx[tunerID]->used = false;

        ++tunerID;
    }

    for (size_t i = 0; i < numDucChannels; ++i)
    {
        this->tunerIdToTx[tunerID] = boost::make_shared<TxObject>();

        this->tunerIdToTx[tunerID]->used = false;

        ++tunerID;
    }

    // Connect the radio to the DDC(s) and DUC(s), if possible
    if (numDdcChannels == 0)
    {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "No DDCs available, RX not possible");
    }
    else if (numDdcChannels == 1)
    {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "Only one DDC channel available, RX possible on one channel only");

        this->graph->connect(this->radio->unique_id(), 0, this->ddcs[0]->unique_id(), 0);

        // Setting radio spp
        try
        {
            this->radio->set_arg("spp", 512, 0);
        }
        catch(...)
        {
            LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to set spp on radio channel 0");
        }

        // Setting ddc spp
        try
        {
            this->ddcs[0]->set_arg("spp", 512, 0);
        }
        catch(...)
        {
            LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to set spp on DDC");
        }

        tunerIdToRx[0]->ddc = this->ddcs[0];
        tunerIdToRx[0]->ddcPort = 0;
        tunerIdToRx[0]->radioChannel = 0;
    }
    else
    {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "Sufficient DDCs for RX on all radios");

        if (this->ddcs.size() == 1)
        {
            this->graph->connect(this->radio->unique_id(), 0, this->ddcs[0]->unique_id(), 0);
            this->graph->connect(this->radio->unique_id(), 1, this->ddcs[0]->unique_id(), 1);

            // Setting radio spps
            try
            {
                this->radio->set_arg("spp", 512, 0);
            }
            catch(...)
            {
                LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to set spp on radio channel 0");
            }

            try
            {
                this->radio->set_arg("spp", 512, 1);
            }
            catch(...)
            {
                LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to set spp on radio channel 1");
            }

            // Setting ddc spps
            try
            {
            	this->ddcs[0]->set_arg("spp", 512, 0);
            }
            catch(...)
            {
                LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to set spp on DDC channel 0");
            }

            try
            {
            	this->ddcs[0]->set_arg("spp", 512, 1);
            }
            catch(...)
            {
                LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to set spp on DDC channel 1");
            }

            tunerIdToRx[0]->ddc = this->ddcs[0];
            tunerIdToRx[0]->ddcPort = 0;
            tunerIdToRx[0]->radioChannel = 0;

            tunerIdToRx[1]->ddc = this->ddcs[0];
            tunerIdToRx[1]->ddcPort = 1;
            tunerIdToRx[1]->radioChannel = 1;
        }
        else
        {
            this->graph->connect(this->radio->unique_id(), 0, this->ddcs[0]->unique_id(), 0);
            this->graph->connect(this->radio->unique_id(), 1, this->ddcs[1]->unique_id(), 0);

            // Setting radio spps
            try
            {
                this->radio->set_arg("spp", 512, 0);
            }
            catch(...)
            {
                LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to set spp on radio channel 0");
            }

            try
            {
                this->radio->set_arg("spp", 512, 1);
            }
            catch(...)
            {
                LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to set spp on radio channel 1");
            }

            // Setting ddc spps
            try
            {
            	this->ddcs[0]->set_arg("spp", 512, 0);
            }
            catch(...) {
                LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to set spp on DDC 0");
            }

            try
            {
            	this->ddcs[1]->set_arg("spp", 512, 0);
            }
            catch(...)
            {
                LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to set spp on DDC 1");
            }

            tunerIdToRx[0]->ddc = this->ddcs[0];
            tunerIdToRx[0]->ddcPort = 0;
            tunerIdToRx[0]->radioChannel = 0;

            tunerIdToRx[1]->ddc = this->ddcs[1];
            tunerIdToRx[1]->ddcPort = 0;
            tunerIdToRx[1]->radioChannel = 1;
        }
    }

    if (numDucChannels == 0)
    {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "No DUCs available, TX not possible");
    }
    else if (numDucChannels == 1)
    {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "Only one DUC channel available, TX possible on one channel only")

        this->graph->connect(this->ducs[0]->unique_id(), 0, this->radio->unique_id(), 0);

        tunerIdToTx[numDdcChannels]->duc = this->ducs[0];
        tunerIdToTx[numDdcChannels]->ducPort = 0;
        tunerIdToTx[numDdcChannels]->radioChannel = 0;
    }
    else
    {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "Sufficient DUCs for TX on all radios");

        if (this->ducs.size() == 1)
        {
            this->graph->connect(this->ducs[0]->unique_id(), 0, this->radio->unique_id(), 0);
            this->graph->connect(this->ducs[0]->unique_id(), 1, this->radio->unique_id(), 1);

            tunerIdToTx[numDdcChannels]->duc = this->ducs[0];
            tunerIdToTx[numDdcChannels]->ducPort = 0;
            tunerIdToTx[numDdcChannels]->radioChannel = 0;

            tunerIdToTx[numDdcChannels + 1]->duc = this->ducs[0];
            tunerIdToTx[numDdcChannels + 1]->ducPort = 1;
            tunerIdToTx[numDdcChannels + 1]->radioChannel = 1;
        }
        else
        {
            this->graph->connect(this->ducs[0]->unique_id(), 0, this->radio->unique_id(), 0);
            this->graph->connect(this->ducs[1]->unique_id(), 0, this->radio->unique_id(), 1);

            tunerIdToTx[numDdcChannels]->duc = this->ducs[0];
            tunerIdToTx[numDdcChannels]->ducPort = 0;
            tunerIdToTx[numDdcChannels]->radioChannel = 0;

            tunerIdToTx[numDdcChannels + 1]->duc = this->ducs[1];
            tunerIdToTx[numDdcChannels + 1]->ducPort = 0;
            tunerIdToTx[numDdcChannels + 1]->radioChannel = 1;
        }
    }

    setNumChannels(numDdcChannels + numDucChannels);

    size_t currentStatus = 0;

    std::vector<size_t> outputPorts = this->radio->get_output_ports();

    for (size_t i = 0; i < numDdcChannels; ++i)
    {
        frontend_tuner_status_struct_struct &fts = this->frontend_tuner_status[currentStatus];

        fts.bandwidth = this->radio->get_output_samp_rate(i);
        fts.center_frequency = this->radio->get_rx_frequency(i);
        fts.gain = this->radio->get_rx_gain(i);
        fts.sample_rate = this->radio->get_output_samp_rate(i);

        ++currentStatus;
    }

    std::vector<size_t> inputPorts = this->radio->get_input_ports();

    for (size_t i = 0; i < numDucChannels; ++i)
    {
        frontend_tuner_status_struct_struct &fts = this->frontend_tuner_status[currentStatus];

        fts.bandwidth = this->radio->get_input_samp_rate(i);
        fts.center_frequency = this->radio->get_tx_frequency(i);
        fts.gain = this->radio->get_tx_gain(i);
        fts.sample_rate = this->radio->get_input_samp_rate(i);
        fts.tuner_type = "TX";

        ++currentStatus;
    }
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

void RFNoC_ProgrammableDevice_i::resetHwLoadStatus(HwLoadStatusStruct &loadStatusStruct)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    unloadHardware(loadStatusStruct);

    loadStatusStruct.hardware_id = this->HARDWARE_ID;
    loadStatusStruct.state = HW_LOAD::INACTIVE;
}

void RFNoC_ProgrammableDevice_i::retrieveRxStream(size_t streamIndex)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Release the old stream if necessary
    if (this->tunerIdToRx[streamIndex]->rxStream)
    {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Releasing old RX stream");

        this->tunerIdToRx[streamIndex]->rxStream.reset();
    }

    uhd::rfnoc::ddc_block_ctrl::sptr ddc = this->tunerIdToRx[streamIndex]->ddc;
    size_t ddcPort = this->tunerIdToRx[streamIndex]->ddcPort;

    // Set the stream arguments
    // Only support short complex for now
    uhd::stream_args_t stream_args("sc16", "sc16");
    uhd::device_addr_t streamer_args;

    streamer_args["block_id"] = ddc->unique_id();

    // Get the spp from the block
    this->tunerIdToRx[streamIndex]->spp = ddc->get_args(ddcPort).cast<size_t>("spp", 1024);

    streamer_args["block_port"] = boost::lexical_cast<std::string>(ddcPort);
    streamer_args["spp"] = boost::lexical_cast<std::string>(this->tunerIdToRx[streamIndex]->spp);

    stream_args.args = streamer_args;

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Using streamer arguments: " << stream_args.args.to_string());

    // Retrieve the RX stream as specified from the device 3
    try
    {
        this->tunerIdToRx[streamIndex]->rxStream = this->usrp->get_rx_stream(stream_args);
    }
    catch(uhd::runtime_error &e)
    {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Failed to retrieve RX stream: " << e.what());
    }
    catch(...)
    {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Unexpected error occurred while retrieving RX stream");
    }
}

void RFNoC_ProgrammableDevice_i::retrieveTxStream(size_t streamIndex)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Release the old stream if necessary
    if (this->tunerIdToTx[streamIndex]->txStream)
    {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Releasing old TX stream");

        this->tunerIdToTx[streamIndex]->txStream.reset();
    }

    uhd::rfnoc::duc_block_ctrl::sptr duc = this->tunerIdToTx[streamIndex]->duc;
    size_t ducPort = this->tunerIdToTx[streamIndex]->ducPort;

    // Set the stream arguments
    // Only support short complex for now
    uhd::stream_args_t stream_args("sc16", "sc16");
    uhd::device_addr_t streamer_args;

    streamer_args["block_id"] = duc->unique_id();

    // Get the spp from the block
    this->tunerIdToTx[streamIndex]->spp = duc->get_args(ducPort).cast<size_t>("spp", 1024);

    streamer_args["block_port"] = boost::lexical_cast<std::string>(ducPort);
    streamer_args["spp"] = boost::lexical_cast<std::string>(this->tunerIdToTx[streamIndex]->spp);

    stream_args.args = streamer_args;

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Using streamer arguments: " << stream_args.args.to_string());

    // Retrieve the TX stream as specified from the device 3
    try
    {
        this->tunerIdToTx[streamIndex]->txStream = this->usrp->get_tx_stream(stream_args);
    }
    catch(uhd::runtime_error &e)
    {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Failed to retrieve TX stream: " << e.what());
    }
    catch(...)
    {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Unexpected error occurred while retrieving TX stream");
    }
}

int RFNoC_ProgrammableDevice_i::rxServiceFunction(size_t streamIndex)
{
    // Perform RX, if necessary
    if (this->tunerIdToRx[streamIndex]->used)
    {
        // Get references to the members
        std::vector<std::complex<short> > &output = this->tunerIdToRx[streamIndex]->output;
        uhd::rx_streamer::sptr &rxStream = this->tunerIdToRx[streamIndex]->rxStream;
        BULKIO::StreamSRI &sri = this->tunerIdToRx[streamIndex]->sri;

        // Push SRI if necessary
        if (this->tunerIdToRx[streamIndex]->updateSRI)
        {
            LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Pushing SRI");

            this->dataShort_out->pushSRI(sri);

            this->tunerIdToRx[streamIndex]->updateSRI = false;
        }

        // Recv from the block
        uhd::rx_metadata_t md;

        LOG_TRACE(RFNoC_ProgrammableDevice_i, "Calling recv on the rx_stream");

        size_t num_rx_samps = rxStream->recv(&output.front(), output.size(), md, 3.0);

        // Check the meta data for error codes
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT)
        {
            LOG_ERROR(RFNoC_ProgrammableDevice_i, "Timeout while streaming");

            return NOOP;
        }
        else if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW)
        {
            LOG_WARN(RFNoC_ProgrammableDevice_i, "Overflow while streaming");
        }
        else if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE)
        {
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

void RFNoC_ProgrammableDevice_i::startRxStream(size_t streamIndex)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    if (not this->tunerIdToRx[streamIndex]->streamStarted)
    {
        // Start continuous streaming
        uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
        stream_cmd.num_samps = 0;
        stream_cmd.stream_now = true;
        stream_cmd.time_spec = uhd::time_spec_t();

        this->tunerIdToRx[streamIndex]->rxStream->issue_stream_cmd(stream_cmd);

        this->tunerIdToRx[streamIndex]->streamStarted = true;
    }
}

void RFNoC_ProgrammableDevice_i::stopRxStream(size_t streamIndex)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    if (this->tunerIdToRx[streamIndex]->streamStarted)
    {
        // Start continuous streaming
        uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);

        this->tunerIdToRx[streamIndex]->rxStream->issue_stream_cmd(stream_cmd);

        this->tunerIdToRx[streamIndex]->streamStarted = false;

        // Run recv until nothing is left
        uhd::rx_metadata_t md;
        int num_post_samps = 0;

        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Emptying receive queue...");

        do
        {
            num_post_samps = this->tunerIdToRx[streamIndex]->rxStream->recv(&this->tunerIdToRx[streamIndex]->output.front(), this->tunerIdToRx[streamIndex]->output.size(), md, 1.0);
        }
        while(num_post_samps and md.error_code == uhd::rx_metadata_t::ERROR_CODE_NONE);

        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Emptied receive queue");
    }
}

void RFNoC_ProgrammableDevice_i::target_deviceChanged(const target_device_struct &oldValue, const target_device_struct &newValue)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to set the target_device while running. Must be set in DCD file.");

    this->target_device = oldValue;
}

int RFNoC_ProgrammableDevice_i::txServiceFunction(size_t streamIndex)
{
    // Perform TX, if necessary
    if (this->tunerIdToTx[streamIndex]->used)
    {
        // Wait on input data
        bulkio::InShortPort::DataTransferType *packet = this->dataShort_in->getPacket(bulkio::Const::BLOCKING);

        if (not packet)
        {
            return NOOP;
        }

        // Respond to the SRI changing
        if (packet->sriChanged)
        {
            LOG_DEBUG(RFNoC_ProgrammableDevice_i, "SRI Changed. Does this device care? Maybe set the DUC");
        }

        // Get references to the members
        uhd::tx_streamer::sptr &txStream = this->tunerIdToTx[streamIndex]->txStream;

        // Prepare the metadata
        uhd::tx_metadata_t md;
        std::complex<short> *block = (std::complex<short> *) packet->dataBuffer.data();
        size_t blockSize = packet->dataBuffer.size() / 2;

        LOG_TRACE(RFNoC_ProgrammableDevice_i, "TX Thread Received " << blockSize << " samples");

        if (blockSize == 0)
        {
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

        if (blockSize != 0 and num_tx_samps == 0)
        {
            LOG_DEBUG(RFNoC_ProgrammableDevice_i, "The TX stream is no longer valid, obtaining a new one");

            retrieveTxStream(streamIndex);
        }

        LOG_TRACE(RFNoC_ProgrammableDevice_i, "TX Thread Sent " << num_tx_samps << " samples");

        // On EOS, forward to the RF-NoC Block
        if (packet->EOS)
        {
            LOG_DEBUG(RFNoC_ProgrammableDevice_i, "EOS");

            md.end_of_burst = true;

            std::vector<std::complex<short> > empty;
            txStream->send(&empty.front(), empty.size(), md);
        }

        delete packet;
    }

    return NORMAL;
}
