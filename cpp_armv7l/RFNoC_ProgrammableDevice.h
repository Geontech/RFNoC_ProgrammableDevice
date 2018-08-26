#ifndef RFNOC_PROGRAMMABLEDEVICE_I_IMPL_H
#define RFNOC_PROGRAMMABLEDEVICE_I_IMPL_H

// Base Include(s)
#include "RFNoC_ProgrammableDevice_prog_base.h"

// FRONTEND Include(s)
#include <frontend/frontend.h>

// RF-NoC RH Include(s)
#include <GenericThreadedComponent.h>
#include <RFNoC_Persona.h>
#include <RFNoC_Programmable.h>

// UHD Include(s)
#include <uhd/device3.hpp>
#include <uhd/rfnoc/ddc_block_ctrl.hpp>
#include <uhd/rfnoc/duc_block_ctrl.hpp>
#include <uhd/rfnoc/radio_ctrl.hpp>

// Local Include(s)
#include "RFNoC_ResourceManager.h"

// Structure(s)
struct RxObject
{
    bool connected;
    uhd::rfnoc::ddc_block_ctrl::sptr ddc;
    size_t ddcPort;
    uhd::rfnoc::block_ctrl_base::sptr downstreamBlock;
    size_t downstreamBlockPort;
    std::vector<std::complex<short> > output;
    size_t radioChannel;
    uhd::rx_streamer::sptr rxStream;
    boost::shared_ptr<RFNoC_RH::GenericThreadedComponent> rxThread;
    size_t spp;
    BULKIO::StreamSRI sri;
    bool streamStarted;
    bool updateSRI;
    bool used;
};

struct TxObject
{
    uhd::rfnoc::duc_block_ctrl::sptr duc;
    size_t ducPort;
    size_t radioChannel;
    uhd::tx_streamer::sptr txStream;
    boost::shared_ptr<RFNoC_RH::GenericThreadedComponent> txThread;
    size_t spp;
    bool used;
};

// Forward Declaration(s)
class RFNoC_ProgrammableDevice_i;

// Type Definition(s)

// Change the template to use the property versions of these structures
// This allows users to see the available loads and perform an allocation
typedef RFNoC_ProgrammableDevice_prog_base<hw_load_request_struct_struct, hw_load_statuses_struct_struct> RFNoC_ProgrammableDevice_prog_base_type;

class RFNoC_ProgrammableDevice_i : public RFNoC_ProgrammableDevice_prog_base_type, public frontend::digital_tuner_delegation, public RFNoC_RH::RFNoC_Programmable
{
    ENABLE_LOGGING

	// Constructor(s) and/or Destructor
    public:
        RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl);
        RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev);
        RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities);
        RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev);

        virtual ~RFNoC_ProgrammableDevice_i();

    // Public Method(s)
    public:
        CORBA::Boolean allocateCapacity(const CF::Properties& capacities)
            throw (
                CF::Device::InvalidState,
                CF::Device::InvalidCapacity,
                CF::Device::InsufficientCapacity,
                CORBA::SystemException);

        bool connectRadioRX(RFNoC_RH::PortHashType portHash, const RFNoC_RH::BlockDescriptor &blockDescriptor);

        bool connectRadioTX(const std::string &allocationId, const RFNoC_RH::BlockDescriptor &blockDescriptor);

        void deallocateCapacity(const CF::Properties& capacities)
            throw (
                CF::Device::InvalidState,
                CF::Device::InvalidCapacity,
                CORBA::SystemException);

		virtual CF::ExecutableDevice::ProcessID_Type execute (const char* name, const CF::Properties& options, const CF::Properties& parameters)
			throw ( CF::ExecutableDevice::ExecuteFail, CF::InvalidFileName, CF::ExecutableDevice::InvalidOptions,
					CF::ExecutableDevice::InvalidParameters, CF::ExecutableDevice::InvalidFunction, CF::Device::InvalidState,
					CORBA::SystemException);

		uhd::rfnoc::graph::sptr getGraph() { return this->graph; }

		void releaseObject() throw (CF::LifeCycle::ReleaseError, CORBA::SystemException);

		virtual void terminate (CF::ExecutableDevice::ProcessID_Type processId)
			throw (CF::Device::InvalidState, CF::ExecutableDevice::InvalidProcess, CORBA::SystemException);

	// Public RFNoC_Programmable Method(s)
	public:
		virtual bool connectBlocks(const RFNoC_RH::BlockDescriptor &outputDescriptor, const RFNoC_RH::BlockDescriptor &inputDescriptor);

		virtual Resource_impl* generateResource(int argc, char* argv[], RFNoC_RH::RFNoC_Component_Constructor_Ptr fnptr, const char* libraryName);

		/*
		 * This method should return an RF-NoC block to the caller. If the port was unset by the
		 * caller, the port used will be placed into blockDescriptor.
		 */
		virtual uhd::rfnoc::block_ctrl_base::sptr getBlock(RFNoC_RH::BlockDescriptor &blockDescriptor);

		/*
		 * This method should return the block descriptor corresponding to the
		 * given port hash.
		 */
		virtual RFNoC_RH::BlockDescriptor getBlockDescriptorFromHash(RFNoC_RH::PortHashType portHash);

		virtual void incomingConnectionAdded(const std::string &resourceId,
											 const std::string &streamId,
											 RFNoC_RH::PortHashType portHash);

		virtual void incomingConnectionRemoved(const std::string &resourceId,
											   const std::string &streamId,
											   RFNoC_RH::PortHashType portHash);

		virtual void outgoingConnectionAdded(const std::string &resourceId,
											 const std::string &connectionId,
											 RFNoC_RH::PortHashType portHash);

		virtual void outgoingConnectionRemoved(const std::string &resourceId,
											   const std::string &connectionId,
											   RFNoC_RH::PortHashType portHash);

		virtual void setPersonaMapping(const std::string &deviceId, RFNoC_RH::RFNoC_Persona *persona);

		virtual void setRxStreamDescriptor(const std::string &resourceId, const RFNoC_RH::StreamDescriptor &streamDescriptor);

		virtual void setTxStreamDescriptor(const std::string &resourceId, const RFNoC_RH::StreamDescriptor &streamDescriptor);

	// Public frontend::digital_tuner_delegation Method(s)
    public:
        std::string getTunerType(const std::string& id);
        bool getTunerDeviceControl(const std::string& id);
        std::string getTunerGroupId(const std::string& id);
        std::string getTunerRfFlowId(const std::string& id);
        CF::Properties* getTunerStatus(const std::string &allocation_id);
        void setTunerCenterFrequency(const std::string& id, double freq);
        double getTunerCenterFrequency(const std::string& id);
        void setTunerBandwidth(const std::string& id, double bw);
        double getTunerBandwidth(const std::string& id);
        void setTunerAgcEnable(const std::string& id, bool enable);
        bool getTunerAgcEnable(const std::string& id);
        void setTunerGain(const std::string& id, float gain);
        float getTunerGain(const std::string& id);
        void setTunerReferenceSource(const std::string& id, long source);
        long getTunerReferenceSource(const std::string& id);
        void setTunerEnable(const std::string& id, bool enable);
        bool getTunerEnable(const std::string& id);
        void setTunerOutputSampleRate(const std::string& id, double sr);
        double getTunerOutputSampleRate(const std::string& id);

    // Protected Method(s)
    protected:
        void constructor();

        int serviceFunction() { return FINISH; }

    // Protected RFNoC_ProgrammableDevice_prog_base_type Method(s)
    protected:
        Device_impl* generatePersona(int argc, char* argv[], ConstructorPtr fnptr, const char* libName);

        virtual bool hwLoadRequestIsValid(const HwLoadRequestStruct& hwLoadRequestStruct);

        bool loadHardware(HwLoadStatusStruct& requestStatus);

        void unloadHardware(const HwLoadStatusStruct& requestStatus);

    // Protected frontend::digital_tuner_delegation Method(s)
    protected:
        ///////////////////////////////
        // Device specific functions //
        ///////////////////////////////
        void deviceEnable(frontend_tuner_status_struct_struct &fts, size_t tuner_id);
        void deviceDisable(frontend_tuner_status_struct_struct &fts, size_t tuner_id);
        bool deviceSetTuning(const frontend_tuner_allocation_struct &request, frontend_tuner_status_struct_struct &fts, size_t tuner_id);
        bool deviceDeleteTuning(frontend_tuner_status_struct_struct &fts, size_t tuner_id);

        ///////////////////////////////
        // Mapping and translation helpers. External string identifiers to internal numerical identifiers
        ///////////////////////////////
        virtual std::string getControlAllocationId(size_t tuner_id);
        virtual std::vector<std::string> getListenerAllocationIds(size_t tuner_id);
        virtual long getTunerMapping(std::string allocation_id);
        std::string createAllocationIdCsv(size_t tuner_id);
        virtual bool removeTunerMapping(size_t tuner_id, std::string allocation_id);
        virtual bool removeTunerMapping(size_t tuner_id);
        virtual void assignListener(const std::string& listen_alloc_id, const std::string& alloc_id);
        virtual void removeListener(const std::string& listen_alloc_id);
        void matchAllocationIdToStreamId(const std::string allocation_id, const std::string stream_id, const std::string port_name="");
        void removeAllocationIdRouting(const size_t tuner_id);
        void removeStreamIdRouting(const std::string stream_id, const std::string allocation_id="");
        virtual void setNumChannels(size_t num);
        virtual void setNumChannels(size_t num, std::string tuner_type);

        // Configure tuner - gets called during allocation
        virtual bool enableTuner(size_t tuner_id, bool enable);
        virtual bool listenerRequestValidation(frontend_tuner_allocation_struct &request, size_t tuner_id);

        // Stream Helpers
        BULKIO::StreamSRI create(std::string &stream_id, frontend_tuner_status_struct_struct &frontend_status, double collector_frequency = -1.0);
        std::string getStreamId(size_t tuner_id);

    // Private Method(s)
    private:
        template <typename CORBAXX> bool addModifyKeyword(BULKIO::StreamSRI *sri, CORBA::String_member id, CORBAXX myValue, bool addOnly = false) {
            CORBA::Any value;
            value <<= myValue;
            unsigned long keySize = sri->keywords.length();
            if (!addOnly) {
                for (unsigned int i = 0; i < keySize; i++) {
                    if (!strcmp(sri->keywords[i].id, id)) {
                        sri->keywords[i].value = value;
                        return true;
                    }
                }
            }
            sri->keywords.length(keySize + 1);
            if (sri->keywords.length() != keySize + 1)
                return false;
            sri->keywords[keySize].id = CORBA::string_dup(id);
            sri->keywords[keySize].value = value;
            return true;
        }

        void clearFlows();

        void connectionAdded(const char *connectionID);

        void connectionRemoved(const char *connectionID);

        void connectionTableChanged(const std::vector<connection_descriptor_struct> &oldValue,
        							const std::vector<connection_descriptor_struct> &newValue);

        void desiredRxChannelsChanged(const unsigned char &oldValue, const unsigned char &newValue);

        void desiredTxChannelsChanged(const unsigned char &oldValue, const unsigned char &newValue);

        void gatherBlocks();

        void initializeRadioChain();

        bool loadBitfile(const std::string &bitfilePath);

        void resetHwLoadStatus(HwLoadStatusStruct &loadStatusStruct);

        void retrieveRxStream(size_t streamIndex);

        void retrieveTxStream(size_t streamIndex);

        int rxServiceFunction(size_t streamIndex);

        void startRxStream(size_t streamIndex);

        void stopRxStream(size_t streamIndex);

        void target_deviceChanged(const target_device_struct &oldValue, const target_device_struct &newValue);

        int txServiceFunction(size_t streamIndex);

        CF::Device::UsageType updateUsageState();

    // Private Constant Member(s)
    private:
		const std::string DEFAULT_BITFILE_PATH;

		const std::string HARDWARE_ID;

		const std::string IDLE_BITFILE_PATH;

    // Private Member(s)
    private:
        // Type Definition(s)
		typedef std::map<std::string, boost::shared_ptr<RxObject> > allocationIdToRxMap;
		typedef std::map<std::string, boost::shared_ptr<TxObject> > allocationIdToTxMap;
		typedef std::map<uhd::rfnoc::block_id_t, uhd::rfnoc::block_ctrl_base::sptr> blockIdToBlockMap;
        typedef std::map<std::string, hw_load_statuses_struct_struct> deviceHwStatusMap;
        typedef boost::mutex::scoped_lock exclusive_lock;
        typedef std::map<CF::ExecutableDevice::ProcessID_Type, std::string> pidDeviceMap;
        typedef std::map<std::string, size_t> string_number_mapping;
        typedef std::map<size_t, boost::shared_ptr<RxObject> > tunerIdToRxMap;
        typedef std::map<size_t, boost::shared_ptr<TxObject> > tunerIdToTxMap;

        std::string activeDeviceId;

        allocationIdToRxMap allocationIdToRx;

        allocationIdToTxMap allocationIdToTx;

        // Provides mapping from unique allocation ID to internal tuner (channel) number
		boost::mutex allocation_id_mapping_lock;
		string_number_mapping allocation_id_to_tuner_id;

		blockIdToBlockMap blocks;

		std::vector<uhd::rfnoc::ddc_block_ctrl::sptr> ddcs;

		std::vector<uhd::rfnoc::duc_block_ctrl::sptr> ducs;

		bool canUnlink;

		deviceHwStatusMap deviceIdToHwStatus;

		frontend::InDigitalTunerPort *DigitalTuner_in;

		uhd::rfnoc::graph::sptr graph;

		std::map<std::string, std::string> listeners;

		pidDeviceMap pidToDeviceId;

        uhd::rfnoc::radio_ctrl::sptr radio;

        boost::shared_ptr<RFNoC_ResourceManager> resourceManager;

        // tuner_allocation_ids is exclusively paired with property frontend_tuner_status.
        // tuner_allocation_ids tracks allocation ids while frontend_tuner_status provides tuner information.
        std::vector<frontend::tunerAllocationIdsStruct> tuner_allocation_ids;

        tunerIdToRxMap tunerIdToRx;

        tunerIdToTxMap tunerIdToTx;

        uhd::device3::sptr usrp;

        uhd::device_addr_t usrpAddress;
};

#endif
