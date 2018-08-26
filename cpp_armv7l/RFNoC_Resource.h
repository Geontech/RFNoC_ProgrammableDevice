#ifndef RFNOC_RESOURCE_H
#define RFNOC_RESOURCE_H

// Local Include(s)
#include "entry_point.h"
#include "RFNoC_ResourceManager.h"

// BULKIO Include(s)
#include <bulkio/bulkio.h>

// OSSIE Include(s)
#include <ossie/debug.h>

// RF-NoC RH Include(s)
#include <RFNoC_Programmable.h>
#include <RFNoC_Utils.h>

// Standard C++ Include(s)
#include <string>
#include <vector>

// UHD Include(s)
#include <uhd/rfnoc/block_id.hpp>
#include <uhd/rfnoc/graph.hpp>

// Logging Macro(s)
#define LOG_TRACE_ID(classname, id, expression)     LOG_TRACE(classname, id << ":" << expression)
#define LOG_DEBUG_ID(classname, id, expression)     LOG_DEBUG(classname, id << ":" << expression)
#define LOG_INFO_ID(classname, id, expression)      LOG_INFO(classname, id << ":" << expression)
#define LOG_WARN_ID(classname, id, expression)      LOG_WARN(classname, id << ":" << expression)
#define LOG_ERROR_ID(classname, id, expression)     LOG_ERROR(classname, id << ":" << expression)
#define LOG_FATAL_ID(classname, id, expression)     LOG_FATAL(classname, id << ":" << expression)

// Enumeration(s)
enum ConnectionType { NONE, FABRIC, RADIO, STREAMER };

// Forward declaration(s)
class RFNoC_ResourceManager;

class RFNoC_Resource
{
    ENABLE_LOGGING

	// Constructor(s) and/or Destructor
    public:
        RFNoC_Resource(const std::string &resourceID, RFNoC_ResourceManager *resourceManager, RFNoC_ProgrammableDevice_i *programmable, uhd::rfnoc::graph::sptr graph);

        virtual ~RFNoC_Resource();

    // Public Method(s)
    public:
        bool connectedToPortWithHash(const RFNoC_RH::PortHashType hash);

        RFNoC_RH::BlockDescriptor getProvidesBlockDescriptor() const;

        std::vector<RFNoC_RH::PortHashType> getProvidesHashes() const;

        RFNoC_RH::BlockDescriptor getUsesBlockDescriptor() const;

        void handleIncomingConnection(const std::string &streamID, const RFNoC_RH::PortHashType portHash);

        bool hasHash(RFNoC_RH::PortHashType hash) const;

        std::string id() const { return this->ID; }

        Resource_impl *instantiate(int argc, char* argv[], ConstructorPtr fnptr, const char* libraryName);

        void newIncomingConnection(const std::string &ID, const RFNoC_RH::PortHashType hash);

        void newOutgoingConnection(const std::string &ID, const RFNoC_RH::PortHashType hash);

        void removedIncomingConnection(const std::string &ID, const RFNoC_RH::PortHashType hash);

        void removedOutgoingConnection(const std::string &ID, const RFNoC_RH::PortHashType hash);

        void setRxStreamer(bool enable);

        void setTxStreamer(bool enable);

        void setBlockDescriptors(const std::vector<RFNoC_RH::BlockDescriptor> &blockDescriptors);

    // Private Member(s)
    private:
        std::vector<RFNoC_RH::BlockDescriptor> blockDescriptors;
        std::vector<RFNoC_RH::PortHashType> connectedPortHashes;
        std::map<std::string, ConnectionType> connectionIdToConnectionType;
        uhd::rfnoc::graph::sptr graph;
        std::string ID;
        bool isRxStreamer;
        bool isTxStreamer;
        RFNoC_ProgrammableDevice_i *programmable;
        std::vector<RFNoC_RH::PortHashType> providesHashes;
        std::vector<BULKIO::ProvidesPortStatisticsProvider_ptr> providesPorts;
        RFNoC_ResourceManager *resourceManager;
        Resource_impl *rhResource;
        std::map<std::string, ConnectionType> streamIdToConnectionType;
        std::vector<BULKIO::UsesPortStatisticsProvider_ptr> usesPorts;
};

#endif
