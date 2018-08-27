#ifndef RFNOC_RESOURCEMANAGER_H
#define RFNOC_RESOURCEMANAGER_H

// Local Include(s)
#include "entry_point.h"
#include "RFNoC_ProgrammableDevice.h"
#include "RFNoC_Resource.h"

// OSSIE Include(s)
#include <ossie/debug.h>
#include <ossie/Device_impl.h>
#include <ossie/Resource_impl.h>

// RF-NoC RH Include(s)
#include <RFNoC_Programmable.h>

// UHD Include(s)
#include <uhd/device3.hpp>
#include <uhd/rfnoc/block_id.hpp>
#include <uhd/rfnoc/graph.hpp>

// Forward Declaration(s)
class RFNoC_Resource;

// Structure(s)
struct IncomingConnection
{
    RFNoC_RH::PortHashType portHash;
    std::string resourceId;
    std::string streamId;
};

class RFNoC_ResourceManager
{
    ENABLE_LOGGING

	// Constructor(s) and/or Destructor
    public:
        RFNoC_ResourceManager(Device_impl *parent, RFNoC_ProgrammableDevice_i *programmable);

        ~RFNoC_ResourceManager();

    // Getter(s) and/or Setters
    public:
        Device_impl* getParent() const { return this->parent; }

    // Public Method(s)
    public:
        Resource_impl* addResource(int argc, char* argv[], ConstructorPtr fnptr, const char* libraryName);

        RFNoC_RH::BlockDescriptor getProvidesBlockDescriptorFromHash(RFNoC_RH::PortHashType hash) const;

        RFNoC_RH::BlockDescriptor getUsesBlockDescriptorFromHash(RFNoC_RH::PortHashType hash) const;

        void registerIncomingConnection(IncomingConnection connection);

        void removeResource(const std::string &resourceID);

        void setBlockDescriptorMapping(const std::string &resourceID, const std::vector<RFNoC_RH::BlockDescriptor> &blockDescriptors);

    // Private Method(s)
    private:
        void connectionHandler();

    // Private Member(s)
    private:
        // Type Definition(s)
        typedef std::map<std::string, boost::shared_ptr<RFNoC_Resource> > RFNoC_ResourceMap;

        boost::condition_variable connectionCondition;
        boost::mutex connectionLock;
        boost::shared_ptr<boost::thread> connectionThread;
        uhd::rfnoc::graph::sptr graph;
        RFNoC_ResourceMap idToResource;
        Device_impl *parent;
        std::vector<IncomingConnection> pendingConnections;
        RFNoC_ProgrammableDevice_i *programmable;
        boost::mutex resourceLock;
};

#endif
