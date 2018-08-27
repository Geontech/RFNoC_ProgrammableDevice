#include "RFNoC_Resource.h"

PREPARE_LOGGING(RFNoC_Resource)

/*
 * Constructor(s) and/or Destructor
 */

RFNoC_Resource::RFNoC_Resource(const std::string &resourceID, RFNoC_ResourceManager *resourceManager, RFNoC_ProgrammableDevice_i *programmable, uhd::rfnoc::graph::sptr graph) :
    graph(graph),
    ID(resourceID),
    isRxStreamer(false),
    isTxStreamer(false),
    programmable(programmable),
    resourceManager(resourceManager),
    rhResource(NULL)
{
    LOG_TRACE_ID(RFNoC_Resource, this->ID, __PRETTY_FUNCTION__);
}

RFNoC_Resource::~RFNoC_Resource()
{
    LOG_TRACE_ID(RFNoC_Resource, this->ID, __PRETTY_FUNCTION__);
}

/*
 * Public Method(s)
 */

bool RFNoC_Resource::connectedToPortWithHash(const RFNoC_RH::PortHashType hash)
{
    return (std::find(this->connectedPortHashes.begin(), this->connectedPortHashes.end(), hash) != this->connectedPortHashes.end());
}

RFNoC_RH::BlockDescriptor RFNoC_Resource::getProvidesBlockDescriptor() const
{
    LOG_TRACE_ID(RFNoC_Resource, this->ID, __PRETTY_FUNCTION__);

    return this->blockDescriptors.front();
}

std::vector<RFNoC_RH::PortHashType> RFNoC_Resource::getProvidesHashes() const
{
    LOG_TRACE_ID(RFNoC_Resource, this->ID, __PRETTY_FUNCTION__);

    return this->providesHashes;
}

RFNoC_RH::BlockDescriptor RFNoC_Resource::getUsesBlockDescriptor() const
{
    LOG_TRACE_ID(RFNoC_Resource, this->ID, __PRETTY_FUNCTION__);

    return this->blockDescriptors.back();
}

void RFNoC_Resource::handleIncomingConnection(const std::string &streamID, const RFNoC_RH::PortHashType portHash)
{
    LOG_TRACE_ID(RFNoC_Resource, this->ID, __PRETTY_FUNCTION__);

    // Try connecting to a resource managed by this persona
    RFNoC_RH::BlockDescriptor blockDescriptor = this->resourceManager->getUsesBlockDescriptorFromHash(portHash);

    if (uhd::rfnoc::block_id_t::is_valid_block_id(blockDescriptor.blockId))
    {
        RFNoC_RH::BlockDescriptor providesBlockDescriptor = getProvidesBlockDescriptor();

        uhd::rfnoc::block_ctrl_base::sptr providesBlockPtr = this->programmable->getBlock(providesBlockDescriptor);

        if (providesBlockPtr->list_upstream_nodes().count(providesBlockDescriptor.port))
        {
            if (providesBlockPtr->list_upstream_nodes().at(providesBlockDescriptor.port).lock()->unique_id() == blockDescriptor.blockId.get())
            {
                LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Already connected to block");

                return;
            }
        }

        this->graph->connect(blockDescriptor.blockId, blockDescriptor.port, providesBlockDescriptor.blockId, providesBlockDescriptor.port);

        this->streamIdToConnectionType[ID] = FABRIC;
    }
    else
    {
        RFNoC_RH::BlockDescriptor providesBlockDescriptor = getProvidesBlockDescriptor();

        if (this->programmable->connectRadioRX(portHash, providesBlockDescriptor))
        {
            LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Successfully connected to RX radio");

            this->streamIdToConnectionType[ID] = RADIO;
        }

        if (streamIdToConnectionType[ID] == NONE)
        {
            LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Could not connect to TX radio. Setting as TX streamer");

            setTxStreamer(true);

            this->streamIdToConnectionType[ID] = STREAMER;
        }
    }
}

bool RFNoC_Resource::hasHash(RFNoC_RH::PortHashType hash) const
{
    LOG_TRACE_ID(RFNoC_Resource, this->ID, __PRETTY_FUNCTION__);

    return (std::find(this->providesHashes.begin(), this->providesHashes.end(), hash) != this->providesHashes.end());
}

Resource_impl* RFNoC_Resource::instantiate(int argc, char* argv[], ConstructorPtr fnptr, const char* libraryName)
{
    LOG_TRACE_ID(RFNoC_Resource, this->ID, __PRETTY_FUNCTION__);

    // Attempt to instantiate the resource
    bool failed = false;

    try
    {
        this->rhResource = fnptr(argc, argv, this->programmable);

        if (not rhResource)
        {
            LOG_ERROR(RFNoC_Resource, "Failed to instantiate RF-NoC resource");
            failed = true;
        }
    }
    catch(...)
    {
        LOG_ERROR(RFNoC_Resource, "Exception occurred while instantiating RF-NoC resource");

        failed = true;
    }

    if (failed)
    {
        throw std::exception();
    }

    // Map the port hashes
    CF::PortSet::PortInfoSequence *portSet = this->rhResource->getPortSet();

    LOG_DEBUG(RFNoC_Resource, this->rhResource->_identifier);

    for (size_t i = 0; i < portSet->length(); ++i)
    {
        CF::PortSet::PortInfoType info = portSet->operator [](i);

        LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Port Name: " << info.name._ptr);
        LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Port Direction: " << info.direction._ptr);
        LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Port Repository: " << info.repid._ptr);

        // Store the provides port hashes
        if (strstr(info.direction._ptr, "Provides") && strstr(info.repid._ptr, "BULKIO"))
        {
            RFNoC_RH::PortHashType hash = info.obj_ptr->_hash(RFNoC_RH::HASH_SIZE);
            BULKIO::ProvidesPortStatisticsProvider_ptr providesPort = BULKIO::ProvidesPortStatisticsProvider::_narrow(this->rhResource->getPort(info.name._ptr));

            LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Adding provides port with hash: " << hash);

            this->providesHashes.push_back(hash);
            this->providesPorts.push_back(providesPort);
        }

        // Store the uses port pointers
        if (strstr(info.direction._ptr, "Uses") && strstr(info.repid._ptr, "BULKIO"))
        {
            BULKIO::UsesPortStatisticsProvider_ptr usesPort = BULKIO::UsesPortStatisticsProvider::_narrow(this->rhResource->getPort(info.name._ptr));

            LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Adding uses port");

            this->usesPorts.push_back(usesPort);
        }
    }

    delete portSet;

    return this->rhResource;
}

void RFNoC_Resource::newIncomingConnection(const std::string &ID, const RFNoC_RH::PortHashType hash)
{
    LOG_TRACE_ID(RFNoC_Resource, this->ID, __PRETTY_FUNCTION__);

    LOG_DEBUG_ID(RFNoC_Resource, this->ID, "New incoming connection with stream ID " << ID << " and hash " << hash);

    // Make sure this isn't a duplicate
    if (this->streamIdToConnectionType.find(ID) != this->streamIdToConnectionType.end())
    {
        LOG_WARN_ID(RFNoC_Resource, this->ID, "That stream ID already exists");

        return;
    }

    this->streamIdToConnectionType[ID] = NONE;

    IncomingConnection connection;
    connection.portHash = hash;
    connection.resourceId = this->ID;
    connection.streamId = ID;

    this->resourceManager->registerIncomingConnection(connection);

    LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Registered incoming connection");
}

void RFNoC_Resource::newOutgoingConnection(const std::string &ID, const RFNoC_RH::PortHashType hash)
{
    LOG_TRACE_ID(RFNoC_Resource, this->ID, __PRETTY_FUNCTION__);

    LOG_DEBUG_ID(RFNoC_Resource, this->ID, "New outgoing connection with connection ID: " << ID);

    // Make sure this isn't a duplicate
    if (this->connectionIdToConnectionType.find(ID) != this->connectionIdToConnectionType.end())
    {
        LOG_WARN_ID(RFNoC_Resource, this->ID, "That connection ID already exists");

        return;
    }

    this->connectionIdToConnectionType[ID] = NONE;

    LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Searching for new connection ID on port with hash: " << hash);

    BULKIO::UsesPortStatisticsProvider_ptr port;

    for (size_t i = 0; i < this->usesPorts.size(); ++i)
    {
        RFNoC_RH::PortHashType otherHash = this->usesPorts[i]->_hash(RFNoC_RH::HASH_SIZE);

        if (hash == otherHash)
        {
            port = this->usesPorts[i];
            break;
        }
    }

    if (CORBA::is_nil(port))
    {
        LOG_WARN_ID(RFNoC_Resource, this->ID, "Could not find port with provided hash");

        return;
    }

    ExtendedCF::UsesConnectionSequence *connections = port->connections();

    for (size_t j = 0; j < connections->length(); ++j)
    {
        ExtendedCF::UsesConnection connection = (*connections)[j];

        // This connection ID matches
        if (ID == connection.connectionId._ptr)
        {
            RFNoC_RH::PortHashType providesHash = connection.port._ptr->_hash(RFNoC_RH::HASH_SIZE);

            LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Found correct connection ID with hash: " << providesHash);

            // First ask the resource manager if it knows about the provides hash
            RFNoC_RH::BlockDescriptor providesBlockDescriptor = this->resourceManager->getProvidesBlockDescriptorFromHash(providesHash);

            // If not, try connecting to the TX radio, and if that fails, set as a streamer
            if (not uhd::rfnoc::block_id_t::is_valid_block_id(providesBlockDescriptor.blockId))
            {
                LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Provides port for this connection is not managed by this RF-NoC Persona. Attempting to connect to TX Radio");

                RFNoC_RH::BlockDescriptor usesBlockDescriptor = getUsesBlockDescriptor();

                if (this->programmable->connectRadioTX(ID, usesBlockDescriptor))
                {
                    LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Successfully connected to TX radio");

                    this->connectionIdToConnectionType[ID] = RADIO;
                }

                if (this->connectionIdToConnectionType[ID] == NONE)
                {
                    LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Could not connect to TX radio. Setting as RX streamer");

                    setRxStreamer(true);

                    this->connectionIdToConnectionType[ID] = STREAMER;
                }
            }
            // If so, connect to the provides RF-NoC block
            else
            {
                LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Provides port for this connection is managed by this RF-NoC Persona. Attempting to connect to " << providesBlockDescriptor.blockId.to_string());

                RFNoC_RH::BlockDescriptor usesBlockDescriptor = getUsesBlockDescriptor();

                this->graph->connect(usesBlockDescriptor.blockId, usesBlockDescriptor.port, providesBlockDescriptor.blockId, providesBlockDescriptor.port);

                this->connectedPortHashes.push_back(providesHash);

                this->connectionIdToConnectionType[ID] = FABRIC;

                LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Successfully connected");
            }

            break;
        }
    }

    delete connections;
}

void RFNoC_Resource::removedIncomingConnection(const std::string &ID, const RFNoC_RH::PortHashType hash)
{
    LOG_TRACE_ID(RFNoC_Resource, this->ID, __PRETTY_FUNCTION__);

    LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Removed incoming connection with stream ID: " << ID);

    // Make sure this isn't a duplicate
    if (this->streamIdToConnectionType.find(ID) == this->streamIdToConnectionType.end())
    {
        LOG_WARN_ID(RFNoC_Resource, this->ID, "That stream ID is not in use");

        return;
    }

    // Respond to the disconnect appropriately
    if (this->streamIdToConnectionType[ID] == NONE)
    {
        LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Unknown connection type");
    }
    else if (this->streamIdToConnectionType[ID] == FABRIC)
    {
        LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Connection was in fabric. Disconnect somehow...");
    }
    else if (this->streamIdToConnectionType[ID] == RADIO)
    {
        LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Connection was in fabric to radio. Disconnect somehow...");
    }
    else if (this->streamIdToConnectionType[ID] == STREAMER)
    {
        LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Connection was a streamer. Issue stream command...");

        setTxStreamer(false);
    }

    // Unmap this connection ID
    this->streamIdToConnectionType.erase(ID);
}

void RFNoC_Resource::removedOutgoingConnection(const std::string &ID, const RFNoC_RH::PortHashType hash)
{
    LOG_TRACE_ID(RFNoC_Resource, this->ID, __PRETTY_FUNCTION__);

    LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Removed outgoing connection with connection ID: " << ID);

    // Make sure this isn't a duplicate
    if (this->connectionIdToConnectionType.find(ID) == this->connectionIdToConnectionType.end())
    {
        LOG_WARN_ID(RFNoC_Resource, this->ID, "That connection ID is not in use");

        return;
    }

    // Respond to the disconnect appropriately
    if (this->connectionIdToConnectionType[ID] == NONE)
    {
        LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Unknown connection type");
    }
    else if (this->connectionIdToConnectionType[ID] == FABRIC)
    {
        LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Connection was in fabric. Disconnect somehow...");
    }
    else if (this->connectionIdToConnectionType[ID] == RADIO)
    {
        LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Connection was in fabric to radio. Disconnect somehow...");
    }
    else if (this->connectionIdToConnectionType[ID] == STREAMER)
    {
        LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Connection was a streamer. Issue stream command...");

        setRxStreamer(false);
    }

    for (std::vector<RFNoC_RH::PortHashType>::iterator it = this->connectedPortHashes.begin(); it != this->connectedPortHashes.end();)
    {
        if (*it == hash)
        {
            it = this->connectedPortHashes.erase(it);
        }
        else
        {
            ++it;
        }
    }

    // Unmap this connection ID
    this->connectionIdToConnectionType.erase(ID);
}

void RFNoC_Resource::setRxStreamer(bool enable)
{
    LOG_TRACE_ID(RFNoC_Resource, this->ID, __PRETTY_FUNCTION__);

    if (enable and not this->isRxStreamer)
    {
        LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Setting resource as RX streamer");

        // TODO: Make this work
        //this->setRxStreamerCb(true);

        this->isRxStreamer = true;
    }
    else if (not enable and this->isRxStreamer)
    {
        LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Unsetting resource as RX streamer");

        // TODO: Make this work
        //this->setRxStreamerCb(false);

        this->isRxStreamer = false;
    }
}

void RFNoC_Resource::setTxStreamer(bool enable)
{
    LOG_TRACE_ID(RFNoC_Resource, this->ID, __PRETTY_FUNCTION__);

    if (enable and not this->isTxStreamer)
    {
        LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Setting resource as TX streamer");

        // TODO: Make this work
        //this->setTxStreamerCb(true);

        this->isTxStreamer = true;
    }
    else if (not enable and this->isTxStreamer)
    {
        LOG_DEBUG_ID(RFNoC_Resource, this->ID, "Unsetting resource as TX streamer");

        // TODO: Make this work
        //this->setTxStreamerCb(false);

        this->isTxStreamer = false;
    }
}

void RFNoC_Resource::setBlockDescriptors(const std::vector<RFNoC_RH::BlockDescriptor> &blockDescriptors)
{
    LOG_TRACE_ID(RFNoC_Resource, this->ID, __PRETTY_FUNCTION__);

    for (size_t i = 0; i < blockDescriptors.size(); ++i)
    {
        std::string blockID = blockDescriptors[i].blockId.get();

        if (blockID.find("Radio") != std::string::npos)
        {
            LOG_ERROR(RFNoC_Resource, "Unable to claim RF-NoC Resource with ID: " << blockID);

            throw std::exception();
        }
    }

    this->blockDescriptors = blockDescriptors;
}
