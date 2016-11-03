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
    HARDWARE_ID("E310"),
    IDLE_BITFILE_PATH("/usr/share/uhd/images/usrp_e3xx_fpga_idle.bit")
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

RFNoC_ProgrammableDevice_i::RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev) :
    RFNoC_ProgrammableDevice_prog_base_type(devMgr_ior, id, lbl, sftwrPrfl, compDev),
    HARDWARE_ID("E310"),
    IDLE_BITFILE_PATH("/usr/share/uhd/images/usrp_e3xx_fpga_idle.bit")
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

RFNoC_ProgrammableDevice_i::RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities) :
    RFNoC_ProgrammableDevice_prog_base_type(devMgr_ior, id, lbl, sftwrPrfl, capacities),
    HARDWARE_ID("E310"),
    IDLE_BITFILE_PATH("/usr/share/uhd/images/usrp_e3xx_fpga_idle.bit")
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

RFNoC_ProgrammableDevice_i::RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev) :
    RFNoC_ProgrammableDevice_prog_base_type(devMgr_ior, id, lbl, sftwrPrfl, capacities, compDev),
    HARDWARE_ID("E310"),
    IDLE_BITFILE_PATH("/usr/share/uhd/images/usrp_e3xx_fpga_idle.bit")
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

RFNoC_ProgrammableDevice_i::~RFNoC_ProgrammableDevice_i()
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);
}

void RFNoC_ProgrammableDevice_i::initialize() throw (CF::LifeCycle::InitializeError, CORBA::SystemException) 
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Set the load requests and statuses pointers to the properties
    setHwLoadRequestsPtr(&hw_load_requests);
    setHwLoadStatusesPtr(&hw_load_statuses);

    // Set the usrp address
    this->usrpAddress["name"] = this->target_device.name;
    this->usrpAddress["no_reload_fpga"] = true;

    if (not this->target_device.serial.empty()) {
        this->usrpAddress["serial"] = this->target_device.serial;
    }

    if (not this->target_device.type.empty()) {
        this->usrpAddress["type"] = this->target_device.type;
    }

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Trying target_device: " << this->usrpAddress.to_string());

    // Register the property change listener
    this->addPropertyListener(this->target_device, this, &RFNoC_ProgrammableDevice_i::target_deviceChanged);

    // Set the usage state to IDLE
    setUsageState(CF::Device::IDLE);
}

CORBA::Boolean RFNoC_ProgrammableDevice_i::allocateCapacity(const CF::Properties& capacities)
    throw (
        CF::Device::InvalidState,
        CF::Device::InvalidCapacity,
        CF::Device::InsufficientCapacity,
        CORBA::SystemException)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);

    RFNoC_ProgrammableDevice_prog_base_type::allocateCapacity(capacities);

    if (this->tuner_allocation_ids.size() != this->frontend_tuner_status.size()) {
        this->tuner_allocation_ids.resize(this->frontend_tuner_status.size());
    }

    CORBA::ULong ii;
    try{
        for (ii = 0; ii < capacities.length(); ++ii) {
            const std::string id = (const char*) capacities[ii].id;
            if (id != "FRONTEND::tuner_allocation" && id != "FRONTEND::listener_allocation"){
                LOG_DEBUG(RFNoC_ProgrammableDevice_i, "UNKNOWN ALLOCATION PROPERTY1");
                continue;
            }
            PropertyInterface* property = getPropertyFromId(id);
            if(!property){
                LOG_DEBUG(RFNoC_ProgrammableDevice_i, "UNKNOWN PROPERTY");
                throw CF::Device::InvalidCapacity("UNKNOWN PROPERTY", capacities);
            }
            try{
                property->setValue(capacities[ii].value);
            }
            catch(const std::logic_error &e){
                LOG_DEBUG(RFNoC_ProgrammableDevice_i, "COULD NOT PARSE CAPACITY: " << e.what());
                throw CF::Device::InvalidCapacity("COULD NOT PARSE CAPACITY", capacities);
            };
            if (id == "FRONTEND::tuner_allocation"){
                // Check allocation_id
                if (frontend_tuner_allocation.allocation_id.empty()) {
                    LOG_INFO(RFNoC_ProgrammableDevice_i,"allocateCapacity: MISSING ALLOCATION_ID");
                    throw CF::Device::InvalidCapacity("MISSING ALLOCATION_ID", capacities);
                }
                // Check if allocation ID has already been used
                if(getTunerMapping(frontend_tuner_allocation.allocation_id) >= 0){
                    LOG_INFO(RFNoC_ProgrammableDevice_i,"allocateCapacity: ALLOCATION_ID ALREADY IN USE: [" << frontend_tuner_allocation.allocation_id << "]");
                    throw frontend::AllocationAlreadyExists("ALLOCATION_ID ALREADY IN USE", capacities);
                }

                // Check if available tuner
                exclusive_lock lock(allocation_id_mapping_lock);

                // Next, try to allocate a new tuner
                for (size_t tuner_id = 0; tuner_id < tuner_allocation_ids.size(); tuner_id++) {
                    if(frontend_tuner_status[tuner_id].tuner_type != frontend_tuner_allocation.tuner_type) {
                        LOG_DEBUG(RFNoC_ProgrammableDevice_i,
                          "allocateCapacity: Requested tuner type '"<<frontend_tuner_allocation.tuner_type <<"' does not match tuner[" << tuner_id << "].tuner_type ("<<frontend_tuner_status[tuner_id].tuner_type<<")");
                        continue;
                    }

                    if(!frontend_tuner_allocation.group_id.empty() && frontend_tuner_allocation.group_id != frontend_tuner_status[tuner_id].group_id ){
                        LOG_DEBUG(RFNoC_ProgrammableDevice_i,
                          "allocateCapacity: Requested group_id '"<<frontend_tuner_allocation.group_id <<"' does not match tuner[" << tuner_id << "].group_id ("<<frontend_tuner_status[tuner_id].group_id<<")");
                        continue;
                    }

                    // special case because allocation is specifying the input stream, which determines the rf_flow_id, etc.
                    if(!frontend_tuner_allocation.rf_flow_id.empty()
                        && frontend_tuner_allocation.rf_flow_id != frontend_tuner_status[tuner_id].rf_flow_id
                        && frontend_tuner_allocation.tuner_type != "CHANNELIZER"){
                        LOG_DEBUG(RFNoC_ProgrammableDevice_i,
                          "allocateCapacity: Requested rf_flow_id '"<<frontend_tuner_allocation.rf_flow_id <<"' does not match tuner[" << tuner_id << "].rf_flow_id ("<<frontend_tuner_status[tuner_id].rf_flow_id<<")");
                        continue;
                    }

                    if(frontend_tuner_allocation.device_control){
                        double orig_bw = frontend_tuner_status[tuner_id].bandwidth;
                        double orig_cf = frontend_tuner_status[tuner_id].center_frequency;
                        double orig_sr = frontend_tuner_status[tuner_id].sample_rate;
                        // pre-load frontend_tuner_status values (just in case the request is filled but the values are not populated)
                        frontend_tuner_status[tuner_id].bandwidth = frontend_tuner_allocation.bandwidth;
                        frontend_tuner_status[tuner_id].center_frequency = frontend_tuner_allocation.center_frequency;
                        frontend_tuner_status[tuner_id].sample_rate = frontend_tuner_allocation.sample_rate;
                        // device control
                        if(!tuner_allocation_ids[tuner_id].control_allocation_id.empty() || !deviceSetTuning(frontend_tuner_allocation, frontend_tuner_status[tuner_id], tuner_id)){
                            if (frontend_tuner_status[tuner_id].bandwidth == frontend_tuner_allocation.bandwidth)
                                frontend_tuner_status[tuner_id].bandwidth = orig_bw;
                            if (frontend_tuner_status[tuner_id].center_frequency == frontend_tuner_allocation.center_frequency)
                                frontend_tuner_status[tuner_id].center_frequency = orig_cf;
                            if (frontend_tuner_status[tuner_id].sample_rate == frontend_tuner_allocation.sample_rate)
                                frontend_tuner_status[tuner_id].sample_rate = orig_sr;
                            // either not available or didn't succeed setting tuning, try next tuner
                            LOG_DEBUG(RFNoC_ProgrammableDevice_i,
                                "allocateCapacity: Tuner["<<tuner_id<<"] is either not available or didn't succeed while setting tuning ");
                            continue;
                        }
                        tuner_allocation_ids[tuner_id].control_allocation_id = frontend_tuner_allocation.allocation_id;
                        allocation_id_to_tuner_id.insert(std::pair<std::string, size_t > (frontend_tuner_allocation.allocation_id, tuner_id));
                        frontend_tuner_status[tuner_id].allocation_id_csv = createAllocationIdCsv(tuner_id);
                    } else {
                        // channelizer allocations must specify device control = true
                        if(frontend_tuner_allocation.tuner_type == "CHANNELIZER" || frontend_tuner_allocation.tuner_type == "TX"){
                            std::ostringstream eout;
                            eout<<frontend_tuner_allocation.tuner_type<<" allocation with device_control=false is invalid.";
                            LOG_DEBUG(RFNoC_ProgrammableDevice_i, eout.str());
                            throw CF::Device::InvalidCapacity(eout.str().c_str(), capacities);
                        }
                        // listener
                        if(tuner_allocation_ids[tuner_id].control_allocation_id.empty() || !listenerRequestValidation(frontend_tuner_allocation, tuner_id)){
                            // either not allocated or can't support listener request
                            LOG_DEBUG(RFNoC_ProgrammableDevice_i,
                                "allocateCapacity: Tuner["<<tuner_id<<"] is either not available or can not support listener request ");
                            continue;
                        }
                        tuner_allocation_ids[tuner_id].listener_allocation_ids.push_back(frontend_tuner_allocation.allocation_id);
                        allocation_id_to_tuner_id.insert(std::pair<std::string, size_t > (frontend_tuner_allocation.allocation_id, tuner_id));
                        frontend_tuner_status[tuner_id].allocation_id_csv = createAllocationIdCsv(tuner_id);
                        this->assignListener(frontend_tuner_allocation.allocation_id,tuner_allocation_ids[tuner_id].control_allocation_id);
                    }
                    // if we've reached here, we found an eligible tuner with correct frequency

                    // check tolerances
                    // only check when sample_rate was not set to don't care)
                    LOG_DEBUG(RFNoC_ProgrammableDevice_i, std::fixed << " allocateCapacity - SR requested: " << frontend_tuner_allocation.sample_rate
                                                                                                       << "  SR got: " << frontend_tuner_status[tuner_id].sample_rate);
                    if( (frontend::floatingPointCompare(frontend_tuner_allocation.sample_rate,0)!=0) &&
                        (frontend::floatingPointCompare(frontend_tuner_status[tuner_id].sample_rate,frontend_tuner_allocation.sample_rate)<0 ||
                                frontend::floatingPointCompare(frontend_tuner_status[tuner_id].sample_rate,frontend_tuner_allocation.sample_rate+frontend_tuner_allocation.sample_rate * frontend_tuner_allocation.sample_rate_tolerance/100.0)>0 )){
                        std::ostringstream eout;
                        eout<<std::fixed<<"allocateCapacity("<<int(tuner_id)<<"): returned sr "<<frontend_tuner_status[tuner_id].sample_rate<<" does not meet tolerance criteria of "<<frontend_tuner_allocation.sample_rate_tolerance<<" percent";
                        LOG_INFO(RFNoC_ProgrammableDevice_i, eout.str());
                        throw std::logic_error(eout.str().c_str());
                    }
                    LOG_DEBUG(RFNoC_ProgrammableDevice_i, std::fixed << " allocateCapacity - BW requested: " << frontend_tuner_allocation.bandwidth
                                                                                                       << "  BW got: " << frontend_tuner_status[tuner_id].bandwidth);
                    // Only check when bandwidth was not set to don't care
                    if( (frontend::floatingPointCompare(frontend_tuner_allocation.bandwidth,0)!=0) &&
                        (frontend::floatingPointCompare(frontend_tuner_status[tuner_id].bandwidth,frontend_tuner_allocation.bandwidth)<0 ||
                                frontend::floatingPointCompare(frontend_tuner_status[tuner_id].bandwidth,frontend_tuner_allocation.bandwidth+frontend_tuner_allocation.bandwidth * frontend_tuner_allocation.bandwidth_tolerance/100.0)>0 )){
                        std::ostringstream eout;
                        eout<<std::fixed<<"allocateCapacity("<<int(tuner_id)<<"): returned bw "<<frontend_tuner_status[tuner_id].bandwidth<<" does not meet tolerance criteria of "<<frontend_tuner_allocation.bandwidth_tolerance<<" percent";
                        LOG_INFO(RFNoC_ProgrammableDevice_i, eout.str());
                        throw std::logic_error(eout.str().c_str());
                    }

                    if(frontend_tuner_allocation.device_control){
                        // enable tuner after successful allocation
                        try {
                            enableTuner(tuner_id,true);
                        } catch(...){
                            std::ostringstream eout;
                            eout<<"allocateCapacity: Failed to enable tuner after allocation";
                            LOG_INFO(RFNoC_ProgrammableDevice_i, eout.str());
                            throw std::logic_error(eout.str().c_str());
                        }
                    }
                    _usageState = updateUsageState();
                    return true;
                }
                // if we made it here, we failed to find an available tuner
                std::ostringstream eout;
                eout<<"allocateCapacity: NO AVAILABLE TUNER. Make sure that the device has an initialized frontend_tuner_status";
                LOG_INFO(RFNoC_ProgrammableDevice_i, eout.str());
                throw std::logic_error(eout.str().c_str());

            } else if (id == "FRONTEND::listener_allocation") {
                // Check validity of allocation_id's
                if (frontend_listener_allocation.existing_allocation_id.empty()){
                    LOG_INFO(RFNoC_ProgrammableDevice_i,"allocateCapacity: MISSING EXISTING ALLOCATION ID");
                    throw CF::Device::InvalidCapacity("MISSING EXISTING ALLOCATION ID", capacities);
                }
                if (frontend_listener_allocation.listener_allocation_id.empty()){
                    LOG_INFO(RFNoC_ProgrammableDevice_i,"allocateCapacity: MISSING LISTENER ALLOCATION ID");
                    throw CF::Device::InvalidCapacity("MISSING LISTENER ALLOCATION ID", capacities);
                }

                exclusive_lock lock(allocation_id_mapping_lock);

                // Check if listener allocation ID has already been used
                if(getTunerMapping(frontend_listener_allocation.listener_allocation_id) >= 0){
                    LOG_INFO(RFNoC_ProgrammableDevice_i,"allocateCapacity: LISTENER ALLOCATION ID ALREADY IN USE: [" << frontend_listener_allocation.listener_allocation_id << "]");
                    throw frontend::AllocationAlreadyExists("LISTENER ALLOCATION ID ALREADY IN USE", capacities);
                }
                // Do not allocate if existing allocation ID does not exist
                long tuner_id = getTunerMapping(frontend_listener_allocation.existing_allocation_id);
                if (tuner_id < 0){
                    LOG_DEBUG(RFNoC_ProgrammableDevice_i,"allocateCapacity: UNKNOWN CONTROL ALLOCATION ID: ["<< frontend_listener_allocation.existing_allocation_id <<"]");
                    throw FRONTEND::BadParameterException("UNKNOWN CONTROL ALLOCATION ID");
                }

                // listener allocations are not permitted for channelizers or TX
                if(frontend_tuner_status[tuner_id].tuner_type == "CHANNELIZER" || frontend_tuner_status[tuner_id].tuner_type == "TX"){
                    std::ostringstream eout;
                    eout<<"allocateCapacity: listener allocations are not permitted for " << std::string(frontend_tuner_status[tuner_id].tuner_type) << " tuner type";
                    LOG_DEBUG(RFNoC_ProgrammableDevice_i, eout.str());
                    throw CF::Device::InvalidCapacity(eout.str().c_str(), capacities);
                }

                tuner_allocation_ids[tuner_id].listener_allocation_ids.push_back(frontend_listener_allocation.listener_allocation_id);
                allocation_id_to_tuner_id.insert(std::pair<std::string, size_t > (frontend_listener_allocation.listener_allocation_id, tuner_id));
                frontend_tuner_status[tuner_id].allocation_id_csv = createAllocationIdCsv(tuner_id);
                this->assignListener(frontend_listener_allocation.listener_allocation_id,frontend_listener_allocation.existing_allocation_id);
                return true;
            }
            else {
                LOG_INFO(RFNoC_ProgrammableDevice_i,"allocateCapacity: UNKNOWN ALLOCATION PROPERTY2");
                throw CF::Device::InvalidCapacity("UNKNOWN ALLOCATION PROPERTY2", capacities);
            }
        }
    }
    catch(const std::logic_error &e) {
        deallocateCapacity(capacities);
        return false;
    }
    catch(frontend::AllocationAlreadyExists &e) {
        // Don't call deallocateCapacity if the allocationId already exists
        //   - Would end up deallocating a valid tuner/listener
        throw static_cast<CF::Device::InvalidCapacity>(e);
    }
    catch(CF::Device::InvalidCapacity &e) {
        deallocateCapacity(capacities);
        throw e;
    }
    catch(FRONTEND::BadParameterException &e) {
        deallocateCapacity(capacities);
        return false;
    }
    catch(...){
        deallocateCapacity(capacities);
        throw;
    };

    return true;
}

void RFNoC_ProgrammableDevice_i::deallocateCapacity(const CF::Properties& capacities)
    throw (
        CF::Device::InvalidState,
        CF::Device::InvalidCapacity,
        CORBA::SystemException)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);
    for (CORBA::ULong ii = 0; ii < capacities.length(); ++ii) {
        try{
            const std::string id = (const char*) capacities[ii].id;
            if (id != "FRONTEND::tuner_allocation" && id != "FRONTEND::listener_allocation"){
                LOG_INFO(RFNoC_ProgrammableDevice_i,"deallocateCapacity: UNKNOWN ALLOCATION PROPERTY");
                continue;
            }
            PropertyInterface* property = getPropertyFromId(id);
            if(!property){
                LOG_INFO(RFNoC_ProgrammableDevice_i,"deallocateCapacity: UNKNOWN PROPERTY");
                throw CF::Device::InvalidCapacity("UNKNOWN PROPERTY", capacities);
            }
            try{
                property->setValue(capacities[ii].value);
            }
            catch(const std::logic_error &e){
                LOG_DEBUG(RFNoC_ProgrammableDevice_i, "COULD NOT PARSE CAPACITY: " << e.what());
                throw CF::Device::InvalidCapacity("COULD NOT PARSE CAPACITY", capacities);
            };
            if (id == "FRONTEND::tuner_allocation"){
                //LOG_DEBUG(FrontendTunerDevice<TunerStatusStructType>,std::string(__PRETTY_FUNCTION__)+" tuner_allocation");
                // Try to remove control of the device
                long tuner_id = getTunerMapping(frontend_tuner_allocation.allocation_id);
                if (tuner_id < 0){
                    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "ALLOCATION_ID NOT FOUND: [" << frontend_tuner_allocation.allocation_id <<"]");
                    throw CF::Device::InvalidCapacity("ALLOCATION_ID NOT FOUND", capacities);
                }
                //LOG_DEBUG(FrontendTunerDevice<TunerStatusStructType>,std::string(__PRETTY_FUNCTION__)+" tuner_id = " << tuner_id);
                if(tuner_allocation_ids[tuner_id].control_allocation_id == frontend_tuner_allocation.allocation_id){
                    //LOG_DEBUG(FrontendTunerDevice<TunerStatusStructType>,std::string(__PRETTY_FUNCTION__)+" deallocating control for tuner_id = " << tuner_id);
                    enableTuner(tuner_id, false);
                    removeTunerMapping(tuner_id);
                    frontend_tuner_status[tuner_id].allocation_id_csv = createAllocationIdCsv(tuner_id);
                }
                else {
                    //LOG_DEBUG(FrontendTunerDevice<TunerStatusStructType>,std::string(__PRETTY_FUNCTION__)+" deallocating listener for tuner_id = " << tuner_id);
                    // send EOS to listener connection only
                    removeTunerMapping(tuner_id,frontend_tuner_allocation.allocation_id);
                    frontend_tuner_status[tuner_id].allocation_id_csv = createAllocationIdCsv(tuner_id);
                }
            }
            else if (id == "FRONTEND::listener_allocation") {
                //LOG_DEBUG(FrontendTunerDevice<TunerStatusStructType>,std::string(__PRETTY_FUNCTION__)+" listener_allocation");
                long tuner_id = getTunerMapping(frontend_listener_allocation.listener_allocation_id);
                if (tuner_id < 0){
                    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "ALLOCATION_ID NOT FOUND: [" << frontend_listener_allocation.listener_allocation_id <<"]");
                    throw CF::Device::InvalidCapacity("ALLOCATION_ID NOT FOUND", capacities);
                }
                //LOG_DEBUG(FrontendTunerDevice<TunerStatusStructType>,std::string(__PRETTY_FUNCTION__)+" tuner_id = " << tuner_id);
                // send EOS to listener connection only
                removeTunerMapping(tuner_id,frontend_listener_allocation.listener_allocation_id);
                frontend_tuner_status[tuner_id].allocation_id_csv = createAllocationIdCsv(tuner_id);
            }
            else {
                LOG_TRACE(RFNoC_ProgrammableDevice_i,"WARNING: UNKNOWN ALLOCATION PROPERTY \""+ std::string(property->name) + "\". IGNORING!");
            }
        }
        catch(...){
            LOG_DEBUG(RFNoC_ProgrammableDevice_i,"ERROR WHEN DEALLOCATING. SKIPPING...");
        }
    }
    _usageState = updateUsageState();

    RFNoC_ProgrammableDevice_prog_base_type::deallocateCapacity(capacities);
}

bool RFNoC_ProgrammableDevice_i::connectRadioRX(const CORBA::ULong &portHash, const uhd::rfnoc::block_id_t &blockToConnect, const size_t &blockPort)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    if (not this->radioChainGraph) {
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

    if (not this->radioChainGraph) {
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

void RFNoC_ProgrammableDevice_i::setHwLoadStatus(const hw_load_status_object &hwLoadStatus)
{
    LOG_INFO(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    hw_load_statuses_struct_struct hwLoadStatusStruct;
    hwLoadStatusStruct.hardware_id = hwLoadStatus.hardware_id;
    hwLoadStatusStruct.load_filepath = hwLoadStatus.load_filepath;
    hwLoadStatusStruct.request_id = hwLoadStatus.request_id;
    hwLoadStatusStruct.requester_id = hwLoadStatus.requester_id;
    hwLoadStatusStruct.state = hwLoadStatus.state;

    this->hw_load_statuses.push_back(hwLoadStatusStruct);
}

Device_impl* RFNoC_ProgrammableDevice_i::generatePersona(int argc, char* argv[], ConstructorPtr personaEntryPoint, const char* libName)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    hwLoadStatusCallback hwLoadStatusCb = boost::bind(&RFNoC_ProgrammableDevice_i::setHwLoadStatus, this, _1);
    connectRadioRXCallback connectionRadioRXCb = boost::bind(&RFNoC_ProgrammableDevice_i::connectRadioRX, this, _1, _2, _3);
    connectRadioTXCallback connectionRadioTXCb = boost::bind(&RFNoC_ProgrammableDevice_i::connectRadioTX, this, _1, _2, _3);

    // Generate the Persona Device
    Device_impl *persona = personaEntryPoint(argc, argv, this, hwLoadStatusCb, connectionRadioRXCb, connectionRadioTXCb, this->usrpAddress);

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
    LOG_INFO(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Load the requested bitfile
    uhd::image_loader::image_loader_args_t image_loader_args;

    image_loader_args.firmware_path = "";
    image_loader_args.fpga_path = requestStatus.load_filepath;
    image_loader_args.load_firmware = false;
    image_loader_args.load_fpga = true;

    uhd::image_loader::load(image_loader_args);

    // Allow some time for setup
    boost::this_thread::sleep(boost::posix_time::seconds(1.0));

    // Attempt to get a reference to the specified device
    try {
        this->usrp = uhd::device3::make(this->usrpAddress);
    } catch(uhd::key_error &e) {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Unable to find a suitable USRP Device 3.");
        LOG_ERROR(RFNoC_ProgrammableDevice_i, e.what());
        return false;
    } catch(...) {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "An error occurred attempting to get a reference to the USRP device.");
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

    // Reset the USRP device
    this->usrp->clear();

    // Load the idle bitfile
    uhd::image_loader::image_loader_args_t image_loader_args;

    image_loader_args.firmware_path = "";
    image_loader_args.fpga_path = this->IDLE_BITFILE_PATH;
    image_loader_args.load_firmware = false;
    image_loader_args.load_fpga = true;

    uhd::image_loader::load(image_loader_args);

    // Clear the USRP pointer
    this->usrp.reset();

    // Clear the graph
    this->radioChainGraph.reset();

    // Clear the map of allocation IDs to DDCs/DUCs
    this->allocationIDToDDC.clear();
    this->allocationIDToDUC.clear();

    // Clear the map of radio IDs to DDCs/DUCs
    this->radioIDToDDC.clear();
    this->radioIDToDUC.clear();

    // Clear the map of tuner IDs to radios
    this->tunerIDToRadio.clear();

    // Clear the map of tuner IDs to radio use status
    this->tunerIDUsed.clear();

    // Clear the radios, DDCs, and DUCs
    this->radios.clear();
    this->ddcs.clear();
    this->ducs.clear();

    // Clear the frontend_tuner_status and related lists
    setNumChannels(0);
    this->rxStatuses.clear();
    this->txStatuses.clear();
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

    return true;
}

void RFNoC_ProgrammableDevice_i::initializeRadioChain()
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    // Grab the radio blocks
    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Searching for radio blocks");
    std::vector<uhd::rfnoc::block_id_t> radioBlockIDs = this->usrp->find_blocks("Radio");

    for (size_t i = 0; i < radioBlockIDs.size(); ++i) {
        uhd::rfnoc::block_id_t blockId = radioBlockIDs[i];

        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Found radio block with ID: " << blockId.to_string());

        uhd::rfnoc::radio_ctrl::sptr radio = this->usrp->get_block_ctrl<uhd::rfnoc::radio_ctrl>(blockId);

        this->radios.push_back(radio);
    }

    // Grab the DDC blocks
    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Searching for DDC blocks");
    std::vector<uhd::rfnoc::block_id_t> ddcBlockIDs = this->usrp->find_blocks("DDC");

    for (size_t i = 0; i < ddcBlockIDs.size(); ++i) {
        uhd::rfnoc::block_id_t blockId = ddcBlockIDs[i];

        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Found DDC block with ID: " << blockId.to_string());

        uhd::rfnoc::ddc_block_ctrl::sptr ddc = this->usrp->get_block_ctrl<uhd::rfnoc::ddc_block_ctrl>(blockId);

        this->ddcs.push_back(ddc);
    }

    // Grab the DUC blocks
    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Searching for DUC blocks");
    std::vector<uhd::rfnoc::block_id_t> ducBlockIDs = this->usrp->find_blocks("DUC");

    for (size_t i = 0; i < ducBlockIDs.size(); ++i) {
        uhd::rfnoc::block_id_t blockId = ducBlockIDs[i];

        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Found DUC block with ID: " << blockId.to_string());

        uhd::rfnoc::duc_block_ctrl::sptr duc = this->usrp->get_block_ctrl<uhd::rfnoc::duc_block_ctrl>(blockId);

        this->ducs.push_back(duc);
    }

    // Determine the number of valid chains
    size_t validRXChains = std::min(this->radios.size(), this->ddcs.size());
    size_t validTXChains = std::min(this->radios.size(), this->ducs.size());

    if (validRXChains != this->radios.size()) {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Not enough DDCs are available for the number of radios");
    }

    if (validTXChains != this->radios.size()) {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Not enough DUCs are available for the number of radios");
    }

    // Determine the number of channels
    size_t numRXChannels = 0;
    size_t numTXChannels = 0;

    // Iterate over the valid RX chains, gathering the number of channels and
    // connecting the radio blocks to the DDC blocks
    for (size_t i = 0; i < validRXChains; ++i) {
        uhd::rfnoc::block_id_t ddcBlockID = this->ddcs[i]->get_block_id();
        std::vector<size_t> outputPorts = this->radios[i]->get_output_ports();
        uhd::rfnoc::block_id_t radioBlockID = this->radios[i]->get_block_id();

        numRXChannels += outputPorts.size();

        for (size_t j = 0; j < outputPorts.size(); ++j) {
            size_t portNumber = outputPorts[j];

            LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Connecting port " << portNumber << " between " << radioBlockID.to_string() << " and " << ddcBlockID.to_string());

            this->radioChainGraph->connect(radioBlockID, portNumber, ddcBlockID, portNumber);

            this->radioIDToDDC[radioBlockID.to_string()] = std::make_pair(this->ddcs[i], j);
        }
    }

    // Iterate over the valid RX chains, gathering the number of channels and
    // connecting the radio blocks to the DDC blocks
    for (size_t i = 0; i < validTXChains; ++i) {
        uhd::rfnoc::block_id_t ducBlockID = this->ducs[i]->get_block_id();
        std::vector<size_t> inputPorts = this->radios[i]->get_input_ports();
        uhd::rfnoc::block_id_t radioBlockID = this->radios[i]->get_block_id();

        numTXChannels += inputPorts.size();

        for (size_t j = 0; j < inputPorts.size(); ++j) {
            size_t portNumber = inputPorts[j];

            LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Connecting port " << portNumber << " between " << ducBlockID.to_string() << " and " << radioBlockID.to_string());

            this->radioChainGraph->connect(ducBlockID, portNumber, radioBlockID, portNumber);

            this->radioIDToDUC[radioBlockID.to_string()] = std::make_pair(this->ducs[i], j);
        }
    }

    setNumChannels(numRXChannels + numTXChannels);

    size_t currentStatus = 0;

    for (size_t i = 0; i < validRXChains; ++i) {
        std::vector<size_t> outputPorts = this->radios[i]->get_output_ports();

        for (size_t j = 0; j < outputPorts.size(); ++j) {
            frontend_tuner_status_struct_struct &fts = this->frontend_tuner_status[currentStatus];

            fts.bandwidth = this->radios[i]->get_output_samp_rate(j);
            fts.center_frequency = this->radios[i]->get_rx_frequency(j);
            fts.sample_rate = this->radios[i]->get_output_samp_rate(j);

            this->tunerIDToRadio[currentStatus] = std::make_pair(this->radios[i], j);
            this->tunerIDUsed[currentStatus] = false;

            ++currentStatus;
            this->rxStatuses.push_back(&fts);
        }
    }

    for (size_t i = 0; i < validTXChains; ++i) {
        std::vector<size_t> inputPorts = this->radios[i]->get_input_ports();

        for (size_t j = 0; j < inputPorts.size(); ++j) {
            frontend_tuner_status_struct_struct &fts = this->frontend_tuner_status[currentStatus];

            fts.bandwidth = this->radios[i]->get_input_samp_rate(j);
            fts.center_frequency = this->radios[i]->get_tx_frequency(j);
            fts.sample_rate = this->radios[i]->get_input_samp_rate(j);
            fts.tuner_type = "TX";

            this->tunerIDToRadio[currentStatus] = std::make_pair(this->radios[i], j);
            this->tunerIDUsed[currentStatus] = false;

            ++currentStatus;
            this->txStatuses.push_back(&fts);
        }
    }
}

void RFNoC_ProgrammableDevice_i::target_deviceChanged(const target_device_struct &oldValue, const target_device_struct &newValue)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    LOG_WARN(RFNoC_ProgrammableDevice_i, "Attempted to set the target_device while running. Must be set in DCD file.");

    this->target_device = oldValue;
}

CF::Device::UsageType RFNoC_ProgrammableDevice_i::updateUsageState() {
    //LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);
    size_t tunerAllocated = 0;
    for (size_t tuner_id = 0; tuner_id < tuner_allocation_ids.size(); tuner_id++) {
        if (!tuner_allocation_ids[tuner_id].control_allocation_id.empty())
            tunerAllocated++;
    }
    // If no tuners are allocated, device is idle
    if (tunerAllocated == 0)
        return CF::Device::IDLE;
    // If all tuners are allocated, device is busy
    if (tunerAllocated == tuner_allocation_ids.size())
        return CF::Device::BUSY;
    // Else, device is active
    return CF::Device::ACTIVE;
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
    std::map<size_t, std::pair<uhd::rfnoc::radio_ctrl::sptr, size_t> >::iterator it;

    it = this->tunerIDToRadio.find(tuner_id);

    if (it == this->tunerIDToRadio.end()) {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Failed to set tuning: Invalid tuner ID");
        return false;
    }

    // Make sure it isn't already in use
    if (this->tunerIDUsed[tuner_id]) {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Failed to set tuning: Requested tuner already in use");
        return false;
    }

    // Attempt to set the radio with the requested values
    uhd::rfnoc::radio_ctrl::sptr radio = it->second.first;
    size_t channel = it->second.second;

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
            actualCF = radio->set_rx_frequency(request.center_frequency, channel);

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
            actualCF = radio->set_tx_frequency(request.center_frequency, channel);

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
        std::pair<uhd::rfnoc::ddc_block_ctrl::sptr, size_t> ddc = this->radioIDToDDC[radio->get_block_id().to_string()];

        this->allocationIDToDDC[request.allocation_id] = ddc;
    } else if (request.tuner_type == "TX") {
        std::pair<uhd::rfnoc::duc_block_ctrl::sptr, size_t> duc = this->radioIDToDUC[radio->get_block_id().to_string()];

        this->allocationIDToDUC[request.allocation_id] = duc;
    }

    // Mark this radio as used
    this->tunerIDUsed[tuner_id];

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

    std::map<size_t, bool>::iterator it = this->tunerIDUsed.find(tuner_id);

    if (it == this->tunerIDUsed.end()) {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Failed to set tuning: Invalid tuner ID");
        return false;
    }

    const std::string allocationId = getControlAllocationId(tuner_id);

    std::map<std::string, std::pair<uhd::rfnoc::ddc_block_ctrl::sptr, size_t> >::iterator ddcIt = this->allocationIDToDDC.find(allocationId);

    if (ddcIt != this->allocationIDToDDC.end()) {
        this->allocationIDToDDC.erase(ddcIt);
    }

    std::map<std::string, std::pair<uhd::rfnoc::duc_block_ctrl::sptr, size_t> >::iterator ducIt = this->allocationIDToDUC.find(allocationId);

    if (ducIt != this->allocationIDToDUC.end()) {
        this->allocationIDToDUC.erase(ducIt);
    }

    this->tunerIDUsed[tuner_id] = false;

    return true;
}

std::string RFNoC_ProgrammableDevice_i::createAllocationIdCsv(size_t tuner_id){
    //LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);
    std::string alloc_id_csv = "";
    // ensure control allocation_id is first in list
    if (!tuner_allocation_ids[tuner_id].control_allocation_id.empty())
        alloc_id_csv = tuner_allocation_ids[tuner_id].control_allocation_id + ",";
    std::vector<std::string>::iterator it = tuner_allocation_ids[tuner_id].listener_allocation_ids.begin();
    for(; it != tuner_allocation_ids[tuner_id].listener_allocation_ids.end(); it++)
        alloc_id_csv += *it + ",";
    if(!alloc_id_csv.empty())
        alloc_id_csv.erase(alloc_id_csv.size()-1);
    return alloc_id_csv;
}

std::string RFNoC_ProgrammableDevice_i::getControlAllocationId(size_t tuner_id){
    return tuner_allocation_ids[tuner_id].control_allocation_id;
}

std::vector<std::string> RFNoC_ProgrammableDevice_i::getListenerAllocationIds(size_t tuner_id){
    return tuner_allocation_ids[tuner_id].listener_allocation_ids;
}

bool RFNoC_ProgrammableDevice_i::enableTuner(size_t tuner_id, bool enable) {
    //LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);

    bool prev_enabled = frontend_tuner_status[tuner_id].enabled;

    // If going from disabled to enabled
    if (!prev_enabled && enable) {
        deviceEnable(frontend_tuner_status[tuner_id], tuner_id);
    }

    // If going from enabled to disabled
    if (prev_enabled && !enable) {

        deviceDisable(frontend_tuner_status[tuner_id], tuner_id);
    }

    return true;
}

bool RFNoC_ProgrammableDevice_i::listenerRequestValidation(frontend_tuner_allocation_struct &request, size_t tuner_id){
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);

    // ensure requested values are non-negative
    if(frontend::floatingPointCompare(request.center_frequency,0)<0 || frontend::floatingPointCompare(request.bandwidth,0)<0 || frontend::floatingPointCompare(request.sample_rate,0)<0 || frontend::floatingPointCompare(request.bandwidth_tolerance,0)<0 || frontend::floatingPointCompare(request.sample_rate_tolerance,0)<0)
        return false;

    // ensure lower end of requested band fits
    //if((request.center_frequency - (request.bandwidth*0.5)) < (frontend_tuner_status[tuner_id].center_frequency - (frontend_tuner_status[tuner_id].bandwidth*0.5))){
    if( frontend::floatingPointCompare((request.center_frequency-(request.bandwidth*0.5)),(frontend_tuner_status[tuner_id].center_frequency-(frontend_tuner_status[tuner_id].bandwidth*0.5))) < 0 ){
        LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__ << " FAILED LOWER END TEST");
        return false;
    }

    // ensure upper end of requested band fits
    //if((request.center_frequency + (request.bandwidth*0.5)) > (frontend_tuner_status[tuner_id].center_frequency + (frontend_tuner_status[tuner_id].bandwidth*0.5))){
    if( frontend::floatingPointCompare((request.center_frequency + (request.bandwidth*0.5)),(frontend_tuner_status[tuner_id].center_frequency + (frontend_tuner_status[tuner_id].bandwidth*0.5))) > 0 ){
        LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__ << " FAILED UPPER END TEST");
        return false;
    }

    // ensure tuner bandwidth meets requested tolerance
    //if(request.bandwidth > frontend_tuner_status[tuner_id].bandwidth)
    if( frontend::floatingPointCompare(request.bandwidth,frontend_tuner_status[tuner_id].bandwidth) > 0 )
        return false;

    //if(request.bandwidth != 0 && (request.bandwidth+(request.bandwidth*request.bandwidth_tolerance/100)) < frontend_tuner_status[tuner_id].bandwidth)
    if( frontend::floatingPointCompare(request.bandwidth,0)!=0 && frontend::floatingPointCompare((request.bandwidth+(request.bandwidth*request.bandwidth_tolerance/100)),frontend_tuner_status[tuner_id].bandwidth) < 0 )
        return false;

    // ensure tuner sample rate meets requested tolerance
    //if(request.sample_rate > frontend_tuner_status[tuner_id].sample_rate)
    if( frontend::floatingPointCompare(request.sample_rate,frontend_tuner_status[tuner_id].sample_rate) > 0 )
        return false;

    //if(request.sample_rate != 0 && (request.sample_rate+(request.sample_rate*request.sample_rate_tolerance/100)) < frontend_tuner_status[tuner_id].sample_rate)
    if(frontend::floatingPointCompare(request.sample_rate,0)!=0 && frontend::floatingPointCompare((request.sample_rate+(request.sample_rate*request.sample_rate_tolerance/100)),frontend_tuner_status[tuner_id].sample_rate) < 0 )
        return false;

    return true;
};

////////////////////////////
//        MAPPING         //
////////////////////////////

long RFNoC_ProgrammableDevice_i::getTunerMapping(std::string allocation_id) {
    //LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);
    long NO_VALID_TUNER = -1;

    string_number_mapping::iterator iter = allocation_id_to_tuner_id.find(allocation_id);
    if (iter != allocation_id_to_tuner_id.end())
        return iter->second;

    return NO_VALID_TUNER;
}

bool RFNoC_ProgrammableDevice_i::removeTunerMapping(size_t tuner_id, std::string allocation_id) {
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);
    removeListener(allocation_id);
    std::vector<std::string>::iterator it = tuner_allocation_ids[tuner_id].listener_allocation_ids.begin();
    while(it != tuner_allocation_ids[tuner_id].listener_allocation_ids.end()){
        if(*it == allocation_id){
            tuner_allocation_ids[tuner_id].listener_allocation_ids.erase(it);
        } else {
            ++it;
        }
    }
    exclusive_lock lock(allocation_id_mapping_lock);
    if(allocation_id_to_tuner_id.erase(allocation_id) > 0)
        return true;
    return false;
}

bool RFNoC_ProgrammableDevice_i::removeTunerMapping(size_t tuner_id) {
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);
    deviceDeleteTuning(frontend_tuner_status[tuner_id], tuner_id);
    removeAllocationIdRouting(tuner_id);

    long cnt = 0;
    exclusive_lock lock(allocation_id_mapping_lock);
    string_number_mapping::iterator it = allocation_id_to_tuner_id.begin();
    while(it != allocation_id_to_tuner_id.end()){
        if(it->second == tuner_id){
            std::string allocation_id = it->first;
            removeListener(allocation_id);
            allocation_id_to_tuner_id.erase(it++);
            cnt++;
        } else {
            ++it;
        }
    }
    /*
    for(std::vector<std::string>::iterator it = tuner_allocation_ids[tuner_id].listener_allocation_ids.begin(); it != tuner_allocation_ids[tuner_id].listener_allocation_ids.end();it++){
        removeListener(*it);
        allocation_id_to_tuner_id.erase(*it);
        cnt++;
    }
    removeListener(tuner_allocation_ids[tuner_id].control_allocation_id);
    allocation_id_to_tuner_id.erase(tuner_allocation_ids[tuner_id].control_allocation_id);
    */
    tuner_allocation_ids[tuner_id].reset();
    return cnt > 0;
}

void RFNoC_ProgrammableDevice_i::assignListener(const std::string& listen_alloc_id, const std::string& alloc_id) {
};

void RFNoC_ProgrammableDevice_i::removeListener(const std::string& listen_alloc_id) {
};

void RFNoC_ProgrammableDevice_i::removeAllocationIdRouting(const size_t tuner_id) {
    std::string allocation_id = getControlAllocationId(tuner_id);
    std::vector<connection_descriptor_struct> old_table = this->connectionTable;
    std::vector<connection_descriptor_struct>::iterator itr = this->connectionTable.begin();
    while (itr != this->connectionTable.end()) {
        if (itr->connection_id == allocation_id) {
            itr = this->connectionTable.erase(itr);
            continue;
        }
        itr++;
    }
    for (std::map<std::string, std::string>::iterator listener=listeners.begin();listener!=listeners.end();listener++) {
        if (listener->second == allocation_id) {
            std::vector<connection_descriptor_struct>::iterator itr = this->connectionTable.begin();
            while (itr != this->connectionTable.end()) {
                if (itr->connection_id == listener->first) {
                    itr = this->connectionTable.erase(itr);
                    continue;
                }
                itr++;
            }
        }
    }
    this->connectionTableChanged(&old_table, &this->connectionTable);
}

/* This sets the number of entries in the frontend_tuner_status struct sequence property
 * as well as the tuner_allocation_ids vector. Call this function during initialization
 */
void RFNoC_ProgrammableDevice_i::setNumChannels(size_t num)
{
    this->setNumChannels(num, "RX_DIGITIZER");
}
/* This sets the number of entries in the frontend_tuner_status struct sequence property
 * as well as the tuner_allocation_ids vector. Call this function during initialization
 */

void RFNoC_ProgrammableDevice_i::setNumChannels(size_t num, std::string tuner_type)
{
    frontend_tuner_status.clear();
    frontend_tuner_status.resize(num);
    tuner_allocation_ids.clear();
    tuner_allocation_ids.resize(num);
    for (std::vector<frontend_tuner_status_struct_struct>::iterator iter=frontend_tuner_status.begin(); iter!=frontend_tuner_status.end(); iter++) {
        iter->enabled = false;
        iter->tuner_type = tuner_type;
    }
}

