// Class Include(s)
#include "RFNoC_ResourceManager.h"

PREPARE_LOGGING(RFNoC_ResourceManager)

/*
 * Constructor(s) and/or Destructor
 */
RFNoC_ResourceManager::RFNoC_ResourceManager(Device_impl *parent, RFNoC_ProgrammableDevice_i *programmable) :
    parent(parent),
	programmable(programmable)
{
    LOG_TRACE(RFNoC_ResourceManager, __PRETTY_FUNCTION__);

    this->graph = this->programmable->getGraph();

    this->connectionThread = boost::make_shared<boost::thread>(boost::bind(&RFNoC_ResourceManager::connectionHandler, this));
}

RFNoC_ResourceManager::~RFNoC_ResourceManager()
{
    LOG_TRACE(RFNoC_ResourceManager, __PRETTY_FUNCTION__);

    this->connectionThread->interrupt();
    this->connectionCondition.notify_one();
    this->connectionThread->join();
    this->connectionThread.reset();
}

/*
 * Public Method(s)
 */

Resource_impl* RFNoC_ResourceManager::addResource(int argc, char* argv[], ConstructorPtr fnptr, const char* libraryName)
{
    LOG_TRACE(RFNoC_ResourceManager, __PRETTY_FUNCTION__);

    boost::mutex::scoped_lock lock(this->resourceLock);

    std::string resourceId;

    for (int i = 0; i < argc; ++i)
    {
        if (strcmp(argv[i], "COMPONENT_IDENTIFIER") == 0)
        {
            resourceId = argv[i+1];

            break;
        }
    }

    LOG_DEBUG(RFNoC_ResourceManager, "Adding Resource with ID: " << resourceId);

    RFNoC_ResourceMap::iterator resourceMapIt = this->idToResource.find(resourceId);

    if (resourceMapIt != this->idToResource.end())
    {
        LOG_WARN(RFNoC_ResourceManager, "Attempted to add a Resource already tracked by the Resource Manager.");

        return NULL;
    }

    // Instantiate the resource
    LOG_DEBUG(RFNoC_ResourceManager, "Instantiating new RFNoC_Resource");

    boost::shared_ptr<RFNoC_Resource> rfNocResource = boost::make_shared<RFNoC_Resource>(resourceId, this, this->programmable, this->graph);

    if (not rfNocResource)
    {
        LOG_ERROR(RFNoC_ResourceManager, "Failed to instantiate new RFNoC_Resource");

        return NULL;
    }

    // Map the RFNoC_Resource
    this->idToResource[resourceId] = rfNocResource;

    // Instantiate the resource
    Resource_impl *resource = rfNocResource->instantiate(argc, argv, fnptr, libraryName);

    if (not resource)
    {
        LOG_ERROR(RFNoC_ResourceManager, "Failed to instantiate REDHAWK Resource");

        this->idToResource.erase(resourceId);

        return NULL;
    }

    return resource;
}

RFNoC_RH::BlockDescriptor RFNoC_ResourceManager::getProvidesBlockDescriptorFromHash(RFNoC_RH::PortHashType hash) const
{
    LOG_TRACE(RFNoC_ResourceManager, __PRETTY_FUNCTION__);

    for (RFNoC_ResourceMap::const_iterator it = this->idToResource.begin(); it != this->idToResource.end(); ++it)
    {
        if (it->second->hasHash(hash))
        {
            return it->second->getProvidesBlockDescriptor();
        }
    }

    return RFNoC_RH::BlockDescriptor();
}

RFNoC_RH::BlockDescriptor RFNoC_ResourceManager::getUsesBlockDescriptorFromHash(RFNoC_RH::PortHashType hash) const
{
    LOG_TRACE(RFNoC_ResourceManager, __PRETTY_FUNCTION__);

    for (RFNoC_ResourceMap::const_iterator it = this->idToResource.begin(); it != this->idToResource.end(); ++it)
    {
        if (it->second->connectedToPortWithHash(hash))
        {
            return it->second->getUsesBlockDescriptor();
        }
    }

    return RFNoC_RH::BlockDescriptor();
}

void RFNoC_ResourceManager::registerIncomingConnection(IncomingConnection connection)
{
    LOG_TRACE(RFNoC_ResourceManager, __PRETTY_FUNCTION__);

    boost::mutex::scoped_lock lock(this->connectionLock);

    LOG_DEBUG(RFNoC_ResourceManager, "Got lock");

    this->pendingConnections.push_back(connection);

    LOG_DEBUG(RFNoC_ResourceManager, "Pushed connection");

    this->connectionCondition.notify_one();

    LOG_DEBUG(RFNoC_ResourceManager, "Notified");
}

void RFNoC_ResourceManager::removeResource(const std::string &resourceID)
{
    LOG_TRACE(RFNoC_ResourceManager, __PRETTY_FUNCTION__);

    boost::mutex::scoped_lock lock(this->resourceLock);

    LOG_DEBUG(RFNoC_ResourceManager, "Removing Resource with ID: " << resourceID);

    if (this->idToResource.find(resourceID) == this->idToResource.end())
    {
        LOG_WARN(RFNoC_ResourceManager, "Attempted to remove a Resource not tracked by the Resource Manager");

        return;
    }

    LOG_DEBUG(RFNoC_ResourceManager, "Unmapping Resource from RFNoC_ResourceManager");

    this->idToResource.erase(resourceID);
}

void RFNoC_ResourceManager::setBlockDescriptorMapping(const std::string &resourceID, const std::vector<RFNoC_RH::BlockDescriptor> &blockDescriptors)
{
    LOG_TRACE(RFNoC_ResourceManager, __PRETTY_FUNCTION__);

    RFNoC_ResourceMap::iterator it = this->idToResource.find(resourceID);

    if (it == this->idToResource.end())
    {
        LOG_WARN(RFNoC_ResourceManager, "Attempted to set Block ID for unknown Resource: " << resourceID);

        return;
    }

    LOG_DEBUG(RFNoC_ResourceManager, "Setting block IDs for Resource");

    it->second->setBlockDescriptors(blockDescriptors);
}

/*
 * Private Method(s)
 */

void RFNoC_ResourceManager::connectionHandler()
{
    LOG_TRACE(RFNoC_ResourceManager, __PRETTY_FUNCTION__);

    while (true)
    {
        LOG_DEBUG(RFNoC_ResourceManager, "Before lock in connectionHandler");

        boost::mutex::scoped_lock lock(this->connectionLock);

        LOG_DEBUG(RFNoC_ResourceManager, "After lock in connectionHandler");

        if (boost::this_thread::interruption_requested())
        {
            LOG_DEBUG(RFNoC_ResourceManager, "Interruption requested in connectionHandler");
            return;
        }

        while (this->pendingConnections.empty())
        {
            LOG_DEBUG(RFNoC_ResourceManager, "Empty, waiting in connectionHandler");

            this->connectionCondition.wait(lock);

            LOG_DEBUG(RFNoC_ResourceManager, "Condition received in connectionHandler");

            if (boost::this_thread::interruption_requested())
            {
                LOG_DEBUG(RFNoC_ResourceManager, "Interruption requested in connectionHandler");
                return;
            }
        }

        LOG_DEBUG(RFNoC_ResourceManager, "Copying over pending connections");

        std::vector<IncomingConnection> copy = this->pendingConnections;

        LOG_DEBUG(RFNoC_ResourceManager, "Clearing pending connections in connectionHandler");

        this->pendingConnections.clear();

        LOG_DEBUG(RFNoC_ResourceManager, "Cleared pending connections in connectionHandler");

        LOG_DEBUG(RFNoC_ResourceManager, "Releasing lock");

        lock.unlock();

        LOG_DEBUG(RFNoC_ResourceManager, "Iterating over copy");

        for (size_t i = 0; i < copy.size(); ++i)
        {
            IncomingConnection connection = copy[i];

            boost::shared_ptr<RFNoC_Resource> resource = this->idToResource[connection.resourceId];

            LOG_DEBUG(RFNoC_ResourceManager, "About to handle incoming connection in connectionHandler");

            resource->handleIncomingConnection(connection.streamId, connection.portHash);

            LOG_DEBUG(RFNoC_ResourceManager, "Handled incoming connection in connectionHandler");
        }
    }
}
