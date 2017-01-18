#ifndef RFNOC_PROGRAMMABLEDEVICE_I_IMPL_H
#define RFNOC_PROGRAMMABLEDEVICE_I_IMPL_H

#include "RFNoC_ProgrammableDevice_prog_base.h"

// Change the template to use the property versions of these structures
// This allows users to see the available loads and perform an allocation
typedef RFNoC_ProgrammableDevice_prog_base<hw_load_request_struct_struct, hw_load_statuses_struct_struct> RFNoC_ProgrammableDevice_prog_base_type;

#include <frontend/frontend.h>
#include <uhd/device3.hpp>
#include <uhd/rfnoc/ddc_block_ctrl.hpp>
#include <uhd/rfnoc/duc_block_ctrl.hpp>
#include <uhd/rfnoc/radio_ctrl.hpp>

#include "GenericThreadedComponent.h"
#include "RFNoC_Persona.h"

// Objects necessary for data flow
struct RxObject {
    uhd::rfnoc::ddc_block_ctrl::sptr ddc;
    size_t ddcPort;
    std::vector<std::complex<short> > output;
    size_t radioChannel;
    uhd::rx_streamer::sptr rxStream;
    GenericThreadedComponent *rxThread;
    size_t spp;
    BULKIO::StreamSRI sri;
    bool streamStarted;
    bool updateSRI;
    bool used;
};

struct TxObject {
    uhd::rfnoc::duc_block_ctrl::sptr duc;
    size_t ducPort;
    size_t radioChannel;
    uhd::tx_streamer::sptr txStream;
    GenericThreadedComponent *txThread;
    size_t spp;
    bool used;
};

class RFNoC_ProgrammableDevice_i;

class RFNoC_ProgrammableDevice_i : public RFNoC_ProgrammableDevice_prog_base_type
{
    ENABLE_LOGGING
    public:
        RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl);
        RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev);
        RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities);
        RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev);
        ~RFNoC_ProgrammableDevice_i();
        int serviceFunction() { return FINISH; }

        void constructor();

        CORBA::Boolean allocateCapacity(const CF::Properties& capacities) throw (CF::Device::InvalidState, CF::Device::InvalidCapacity, CF::Device::InsufficientCapacity, CORBA::SystemException);
        void deallocateCapacity(const CF::Properties& capacities) throw (CF::Device::InvalidState, CF::Device::InvalidCapacity, CORBA::SystemException);

        virtual CF::ExecutableDevice::ProcessID_Type execute (const char* name, const CF::Properties& options, const CF::Properties& parameters)
            throw ( CF::ExecutableDevice::ExecuteFail, CF::InvalidFileName, CF::ExecutableDevice::InvalidOptions,
                    CF::ExecutableDevice::InvalidParameters, CF::ExecutableDevice::InvalidFunction, CF::Device::InvalidState,
                    CORBA::SystemException);
        virtual void terminate (CF::ExecutableDevice::ProcessID_Type processId)
            throw ( CF::Device::InvalidState, CF::ExecutableDevice::InvalidProcess, CORBA::SystemException);

        void releaseObject() throw (CF::LifeCycle::ReleaseError, CORBA::SystemException);

        bool connectRadioRX(const CORBA::ULong &portHash, const uhd::rfnoc::block_id_t &blockToConnect, const size_t &blockPort);
        bool connectRadioTX(const std::string &allocationID, const uhd::rfnoc::block_id_t &blockToConnect, const size_t &blockPort);
        uhd::device3::sptr getUsrp() { return this->usrp; }
        void setHwLoadStatus(const std::string &deviceID, const hw_load_status_object &hwLoadStatus);

    protected:
        Device_impl* generatePersona(int argc, char* argv[], ConstructorPtr fnptr, const char* libName);
        bool loadHardware(HwLoadStatusStruct& requestStatus);
        void unloadHardware(const HwLoadStatusStruct& requestStatus);

        virtual bool hwLoadRequestIsValid(const HwLoadRequestStruct& hwLoadRequestStruct);

    private:
        void initializeRadioChain();

        void connectionAdded(const char *connectionID);
        void connectionRemoved(const char *connectionID);

        void desiredRxChannelsChanged(const unsigned char &oldValue, const unsigned char &newValue);
        void desiredTxChannelsChanged(const unsigned char &oldValue, const unsigned char &newValue);
        void target_deviceChanged(const target_device_struct &oldValue, const target_device_struct &newValue);

        int rxServiceFunction(size_t streamIndex);
        int txServiceFunction(size_t streamIndex);

        void setGetBlockInfoFromHash(const std::string &deviceID, getBlockInfoFromHashCallback getBlockInfoFromHashCb);

    protected:
        typedef std::map<std::string, size_t> string_number_mapping;
        typedef boost::mutex::scoped_lock exclusive_lock;

        // tuner_allocation_ids is exclusively paired with property frontend_tuner_status.
        // tuner_allocation_ids tracks allocation ids while frontend_tuner_status provides tuner information.
        std::vector<frontend::tunerAllocationIdsStruct> tuner_allocation_ids;

        // Provides mapping from unique allocation ID to internal tuner (channel) number
        string_number_mapping allocation_id_to_tuner_id;
        boost::mutex allocation_id_mapping_lock;

        CF::Device::UsageType updateUsageState();

        void connectionTableChanged(const std::vector<connection_descriptor_struct> &oldValue, const std::vector<connection_descriptor_struct> &newValue);

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

    private:
        void clearFlows();
        BULKIO::StreamSRI create(std::string &stream_id, frontend_tuner_status_struct_struct &frontend_status, double collector_frequency = -1.0);
        std::string getStreamId(size_t tuner_id);
        bool loadBitfile(const std::string &bitfilePath);
        void retrieveRxStream(size_t streamIndex);
        void retrieveTxStream(size_t streamIndex);
        void startRxStream(size_t streamIndex);
        void stopRxStream(size_t streamIndex);

    private:
        ////////////////////////////
        // Other helper functions //
        ////////////////////////////
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

    private:
        // Typedefs
        typedef std::map<std::string, hw_load_statuses_struct_struct> deviceHwStatusMap;
        typedef std::map<std::string, getBlockInfoFromHashCallback> deviceGetBlockInfoMap;
        typedef std::map<CF::ExecutableDevice::ProcessID_Type, std::string> pidDeviceMap;

        // Constants
        const std::string DEFAULT_BITFILE_PATH;
        const std::string HARDWARE_ID;
        const std::string IDLE_BITFILE_PATH;

        // RF-NoC Members
        uhd::rfnoc::radio_ctrl::sptr radio;
        uhd::rfnoc::graph::sptr radioChainGraph;
        uhd::device3::sptr usrp;

        // UHD Members
        uhd::device_addr_t usrpAddress;

        // Front End Members
        std::map<std::string, std::string> listeners;

        // Programmable Members
        std::string activeDeviceID;
        std::map<std::string, RxObject *> allocationIDToRx;
        std::map<std::string, TxObject *> allocationIDToTx;
        bool canUnlink;
        deviceGetBlockInfoMap deviceIDToGetBlockInfo;
        deviceHwStatusMap deviceIDToHwStatus;
        pidDeviceMap pidToDeviceID;
        std::map<size_t, RxObject *> tunerIDToRx;
        std::map<size_t, TxObject *> tunerIDToTx;
};

#endif // RFNOC_PROGRAMMABLEDEVICE_I_IMPL_H
