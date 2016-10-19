#ifndef RFNOC_PROGRAMMABLEDEVICE_I_IMPL_H
#define RFNOC_PROGRAMMABLEDEVICE_I_IMPL_H

#include "RFNoC_ProgrammableDevice_prog_base.h"

// Change the template to use the property versions of these structures
// This allows users to see the available loads and perform an allocation
typedef RFNoC_ProgrammableDevice_prog_base<hw_load_request_struct_struct, hw_load_statuses_struct_struct> RFNoC_ProgrammableDevice_prog_base_type;

#include <frontend/frontend.h>
#include <uhd/device3.hpp>

#include "HwLoadStatus.h"

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
        int serviceFunction();
        void initialize() throw (CF::LifeCycle::InitializeError, CORBA::SystemException);

        void setHwLoadStatus(const hw_load_status_object &hwLoadStatus);

    protected:
        Device_impl* generatePersona(int argc, char* argv[], ConstructorPtr fnptr, const char* libName);
        bool loadHardware(HwLoadStatusStruct& requestStatus);
        void unloadHardware(const HwLoadStatusStruct& requestStatus);

        virtual bool hwLoadRequestIsValid(const HwLoadRequestStruct& hwLoadRequestStruct);

    private:
        void initializeRadios();
        std::vector<std::string> listNoCBlocks();

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
        virtual void removeAllocationIdRouting(const size_t tuner_id);
        virtual void setNumChannels(size_t num);
        virtual void setNumChannels(size_t num, std::string tuner_type);

        // Configure tuner - gets called during allocation
        virtual bool enableTuner(size_t tuner_id, bool enable);
        virtual bool listenerRequestValidation(frontend_tuner_allocation_struct &request, size_t tuner_id);

    private:
        bool frontend_listener_allocation_alloc(const frontend_listener_allocation_struct &newAllocation);
        void frontend_listener_allocation_dealloc(const frontend_listener_allocation_struct &newDeallocation);

        bool frontend_tuner_allocation_alloc(const frontend_tuner_allocation_struct &newAllocation);
        void frontend_tuner_allocation_dealloc(const frontend_tuner_allocation_struct &newDeallocation);

    private:
        const std::string HARDWARE_ID;
        std::map<std::string, std::string> listeners;
        std::vector<std::string> radioIDs;
        std::vector<frontend_tuner_status_struct_struct *> rxStatuses;
        std::vector<frontend_tuner_status_struct_struct *> txStatuses;
        uhd::device3::sptr usrp;
};

#endif // RFNOC_PROGRAMMABLEDEVICE_I_IMPL_H
