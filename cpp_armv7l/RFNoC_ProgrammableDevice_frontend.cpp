#include "RFNoC_ProgrammableDevice.h"

CORBA::Boolean RFNoC_ProgrammableDevice_i::allocateCapacity(const CF::Properties& capacities)
    throw (
        CF::Device::InvalidState,
        CF::Device::InvalidCapacity,
        CF::Device::InsufficientCapacity,
        CORBA::SystemException)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);

    bool retValue;

    retValue = RFNoC_ProgrammableDevice_prog_base_type::allocateCapacity(capacities);

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
        return retValue;
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
        return retValue;
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

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Finished FEI deallocation, calling prog_base allocation");
    RFNoC_ProgrammableDevice_prog_base_type::deallocateCapacity(capacities);
    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Finished prog_base deallocation");
}

CF::Device::UsageType RFNoC_ProgrammableDevice_i::updateUsageState()
{
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

void RFNoC_ProgrammableDevice_i::connectionTableChanged(const std::vector<connection_descriptor_struct> &oldValue, const std::vector<connection_descriptor_struct> &newValue)
{
    this->dataShort_out->updateConnectionFilter(newValue);
}

std::string RFNoC_ProgrammableDevice_i::createAllocationIdCsv(size_t tuner_id)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);
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

std::string RFNoC_ProgrammableDevice_i::getControlAllocationId(size_t tuner_id)
{
    return tuner_allocation_ids[tuner_id].control_allocation_id;
}

std::vector<std::string> RFNoC_ProgrammableDevice_i::getListenerAllocationIds(size_t tuner_id)
{
    return tuner_allocation_ids[tuner_id].listener_allocation_ids;
}

bool RFNoC_ProgrammableDevice_i::enableTuner(size_t tuner_id, bool enable)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);

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

bool RFNoC_ProgrammableDevice_i::listenerRequestValidation(frontend_tuner_allocation_struct &request, size_t tuner_id)
{
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

long RFNoC_ProgrammableDevice_i::getTunerMapping(std::string allocation_id)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);
    long NO_VALID_TUNER = -1;

    string_number_mapping::iterator iter = allocation_id_to_tuner_id.find(allocation_id);
    if (iter != allocation_id_to_tuner_id.end())
        return iter->second;

    return NO_VALID_TUNER;
}

bool RFNoC_ProgrammableDevice_i::removeTunerMapping(size_t tuner_id, std::string allocation_id)
{
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

bool RFNoC_ProgrammableDevice_i::removeTunerMapping(size_t tuner_id)
{
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

void RFNoC_ProgrammableDevice_i::assignListener(const std::string& listen_alloc_id, const std::string& alloc_id)
{
}

void RFNoC_ProgrammableDevice_i::removeListener(const std::string& listen_alloc_id)
{
}

void RFNoC_ProgrammableDevice_i::matchAllocationIdToStreamId(const std::string allocation_id, const std::string stream_id, const std::string port_name)
{
    if (port_name != "") {
        for (std::vector<connection_descriptor_struct>::iterator prop_itr = this->connectionTable.begin(); prop_itr!=this->connectionTable.end(); prop_itr++) {
            if ((*prop_itr).port_name != port_name)
                continue;
            if ((*prop_itr).stream_id != stream_id)
                continue;
            if ((*prop_itr).connection_id != allocation_id)
                continue;
            // all three match. This is a repeat
            return;
        }
        std::vector<connection_descriptor_struct> old_table = this->connectionTable;
        connection_descriptor_struct tmp;
        tmp.connection_id = allocation_id;
        tmp.port_name = port_name;
        tmp.stream_id = stream_id;
        this->connectionTable.push_back(tmp);
        this->connectionTableChanged(old_table, this->connectionTable);
        return;
    }
    std::vector<connection_descriptor_struct> old_table = this->connectionTable;
    connection_descriptor_struct tmp;
    tmp.connection_id = allocation_id;
    tmp.port_name = "dataShort_out";
    tmp.stream_id = stream_id;
    this->connectionTable.push_back(tmp);
    this->connectionTableChanged(old_table, this->connectionTable);
}

void RFNoC_ProgrammableDevice_i::removeAllocationIdRouting(const size_t tuner_id)
{
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
    this->connectionTableChanged(old_table, this->connectionTable);
}

void RFNoC_ProgrammableDevice_i::removeStreamIdRouting(const std::string stream_id, const std::string allocation_id)
{
    std::vector<connection_descriptor_struct> old_table = this->connectionTable;
    std::vector<connection_descriptor_struct>::iterator itr = this->connectionTable.begin();
    while (itr != this->connectionTable.end()) {
        if (allocation_id == "") {
            if (itr->stream_id == stream_id) {
                itr = this->connectionTable.erase(itr);
                continue;
            }
        } else {
            if ((itr->stream_id == stream_id) and (itr->connection_id == allocation_id)) {
                itr = this->connectionTable.erase(itr);
                continue;
            }
        }
        itr++;
    }
    for (std::map<std::string, std::string>::iterator listener=listeners.begin();listener!=listeners.end();listener++) {
        if (listener->second == allocation_id) {
            std::vector<connection_descriptor_struct>::iterator itr = this->connectionTable.begin();
            while (itr != this->connectionTable.end()) {
                if ((itr->connection_id == listener->first) and (itr->stream_id == stream_id)) {
                    itr = this->connectionTable.erase(itr);
                    continue;
                }
                itr++;
            }
        }
    }
    this->connectionTableChanged(old_table, this->connectionTable);
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

/*************************************************************
Functions servicing the tuner control port
*************************************************************/
std::string RFNoC_ProgrammableDevice_i::getTunerType(const std::string& allocation_id)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__ << " allocation_id=" << allocation_id);

    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].tuner_type;
}

bool RFNoC_ProgrammableDevice_i::getTunerDeviceControl(const std::string& allocation_id)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__ << " allocation_id=" << allocation_id);

    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    if (this->getControlAllocationId(idx) == allocation_id)
        return true;
    return false;
}

std::string RFNoC_ProgrammableDevice_i::getTunerGroupId(const std::string& allocation_id)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__ << " allocation_id=" << allocation_id);

    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].group_id;
}

std::string RFNoC_ProgrammableDevice_i::getTunerRfFlowId(const std::string& allocation_id)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__ << " allocation_id=" << allocation_id);

    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].rf_flow_id;
}

void RFNoC_ProgrammableDevice_i::setTunerCenterFrequency(const std::string& allocation_id, double freq)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__ << " allocation_id=" << allocation_id << " freq=" << freq);

    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    if(allocation_id != getControlAllocationId(idx)){
        std::ostringstream msg;
        msg << "setTunerCenterFrequency|ID (" << allocation_id << ") does not have authorization to modify tuner.";
        LOG_WARN(RFNoC_ProgrammableDevice_i,msg.str());
        throw FRONTEND::FrontendException(msg.str().c_str());
    }

    try {
        try {
            if (not frontend::validateRequest(70e6, 6000e6, freq)) {
                std::ostringstream msg;
                msg << "setTunerCenterFrequency|Invalid center frequency (" << freq <<")";
                LOG_WARN(RFNoC_ProgrammableDevice_i,msg.str() );
                throw FRONTEND::BadParameterException(msg.str().c_str());
            }
        } catch (FRONTEND::BadParameterException) {
            throw;
        } catch (...) {
            std::ostringstream msg;
            msg << "setTunerCenterFrequency|Could not retrieve tuner_id to radio channel number mapping";
            LOG_ERROR(RFNoC_ProgrammableDevice_i,msg.str());
            throw FRONTEND::FrontendException(msg.str().c_str());
        }

        if (frontend_tuner_status[idx].tuner_type == "RX_DIGITIZER") {
            std::map<std::string, RxObject *>::iterator it = this->allocationIDToRx.find(allocation_id);

            if (it == this->allocationIDToRx) {
                std::ostringstream msg;
                msg << "setTunerCenterFrequency|ID (" << allocation_id << ") does not map to RX object.";
                LOG_ERROR(RFNoC_ProgrammableDevice_i,msg.str());
                throw FRONTEND::BadParameterException(msg.str().c_str());
            }

            // If the freq has changed (change in stream) or the tuner is disabled, then set it as disabled
            bool is_tuner_enabled = frontend_tuner_status[idx].enabled;

            if (is_tuner_enabled && this->radio->get_rx_frequency(it->second->radioChannel) != freq) {
                this->radio->set_rx_streamer(false, it->second->radioChannel);
            }

            // set hw with new value and update status
            frontend_tuner_status[idx].center_frequency = this->radio->set_rx_frequency(freq, it->second->radioChannel);

            // update status from hw
            it->second->updateSRI = true;

            // re-enable
            if (is_tuner_enabled) {
                this->radio->set_rx_streamer(true, it->second->radioChannel);
            }
        } else if (frontend_tuner_status[idx].tuner_type == "TX") {
            std::map<std::string, TxObject *>::iterator it = this->allocationIDToTx.find(allocation_id);

            if (it == this->allocationIDToTx) {
                std::ostringstream msg;
                msg << "setTunerCenterFrequency|ID (" << allocation_id << ") does not map to TX object.";
                LOG_ERROR(RFNoC_ProgrammableDevice_i,msg.str());
                throw FRONTEND::BadParameterException(msg.str().c_str());
            }

            // If the freq has changed (change in stream) or the tuner is disabled, then set it as disabled
            bool is_tuner_enabled = frontend_tuner_status[idx].enabled;

            if (is_tuner_enabled && this->radio->get_tx_frequency(it->second->radioChannel) != freq) {
                this->radio->set_tx_streamer(false, it->second->radioChannel);
            }

            // set hw with new value and update status
            frontend_tuner_status[idx].center_frequency = this->radio->set_tx_frequency(freq, it->second->radioChannel);

            // re-enable
            if (is_tuner_enabled) {
                this->radio->set_tx_streamer(true, it->second->radioChannel);
            }
        } else {
            std::ostringstream msg;
            msg << "setTunerCenterFrequency|Invalid tuner type. Must be RX_DIGITIZER or TX";
            LOG_ERROR(RFNoC_ProgrammableDevice_i,msg.str());
            throw FRONTEND::BadParameterException(msg.str().c_str());
        }
    } catch (std::exception& e) {
        std::ostringstream msg;
        msg << "setTunerCenterFrequency|Exception: " << e.what();
        LOG_WARN(RFNoC_ProgrammableDevice_i,msg.str());
        throw FRONTEND::FrontendException(msg.str().c_str());
    }
}

double RFNoC_ProgrammableDevice_i::getTunerCenterFrequency(const std::string& allocation_id)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);

    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].center_frequency;
}

void RFNoC_ProgrammableDevice_i::setTunerBandwidth(const std::string& allocation_id, double bw)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);
    throw FRONTEND::NotSupportedException("setTunerBandwidth not supported");
}

double RFNoC_ProgrammableDevice_i::getTunerBandwidth(const std::string& allocation_id)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);

    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].bandwidth;
}

void RFNoC_ProgrammableDevice_i::setTunerAgcEnable(const std::string& allocation_id, bool enable)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);
    throw FRONTEND::NotSupportedException("setTunerAgcEnable not supported");
}

bool RFNoC_ProgrammableDevice_i::getTunerAgcEnable(const std::string& allocation_id)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);
    throw FRONTEND::NotSupportedException("getTunerAgcEnable not supported");
}

void RFNoC_ProgrammableDevice_i::setTunerGain(const std::string& allocation_id, float gain)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__ << " allocation_id=" << allocation_id << " gain=" << gain);

    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    if(allocation_id != getControlAllocationId(idx)){
        std::ostringstream msg;
        msg << "setTunerGain|ID (" << allocation_id << ") does not have authorization to modify tuner.";
        LOG_WARN(RFNoC_ProgrammableDevice_i,msg.str());
        throw FRONTEND::FrontendException(msg.str().c_str());
    }

    try {
        try {
            if (not frontend::validateRequest(0, 89.5, gain)) {
                std::ostringstream msg;
                msg << "setTunerGain|Invalid gain (" << gain <<")";
                LOG_WARN(RFNoC_ProgrammableDevice_i,msg.str() );
                throw FRONTEND::BadParameterException(msg.str().c_str());
            }
        } catch (FRONTEND::BadParameterException) {
            throw;
        } catch (...) {
            std::ostringstream msg;
            msg << "setTunerGain|Could not retrieve tuner_id to radio channel number mapping";
            LOG_ERROR(RFNoC_ProgrammableDevice_i,msg.str());
            throw FRONTEND::FrontendException(msg.str().c_str());
        }

        if (frontend_tuner_status[idx].tuner_type == "RX_DIGITIZER") {
            std::map<std::string, RxObject *>::iterator it = this->allocationIDToRx.find(allocation_id);

            if (it == this->allocationIDToRx) {
                std::ostringstream msg;
                msg << "setTunerGain|ID (" << allocation_id << ") does not map to RX object.";
                LOG_ERROR(RFNoC_ProgrammableDevice_i,msg.str());
                throw FRONTEND::BadParameterException(msg.str().c_str());
            }

            // If the freq has changed (change in stream) or the tuner is disabled, then set it as disabled
            bool is_tuner_enabled = frontend_tuner_status[idx].enabled;

            if (is_tuner_enabled && this->radio->get_rx_gain(it->second->radioChannel) != gain) {
                this->radio->set_rx_streamer(false, it->second->radioChannel);
            }

            // set hw with new value and update status
            frontend_tuner_status[idx].gain = this->radio->set_rx_gain(gain, it->second->radioChannel);

            // update status from hw
            it->second->updateSRI = true;

            // re-enable
            if (is_tuner_enabled) {
                this->radio->set_rx_streamer(true, it->second->radioChannel);
            }
        } else if (frontend_tuner_status[idx].tuner_type == "TX") {
            std::map<std::string, TxObject *>::iterator it = this->allocationIDToTx.find(allocation_id);

            if (it == this->allocationIDToTx) {
                std::ostringstream msg;
                msg << "setTunerGain|ID (" << allocation_id << ") does not map to TX object.";
                LOG_ERROR(RFNoC_ProgrammableDevice_i,msg.str());
                throw FRONTEND::BadParameterException(msg.str().c_str());
            }

            // If the freq has changed (change in stream) or the tuner is disabled, then set it as disabled
            bool is_tuner_enabled = frontend_tuner_status[idx].enabled;

            if (is_tuner_enabled && this->radio->get_tx_gain(it->second->radioChannel) != gain) {
                this->radio->set_tx_streamer(false, it->second->radioChannel);
            }

            // set hw with new value and update status
            frontend_tuner_status[idx].gain = this->radio->set_tx_gain(gain, it->second->radioChannel);

            // re-enable
            if (is_tuner_enabled) {
                this->radio->set_tx_streamer(true, it->second->radioChannel);
            }
        } else {
            std::ostringstream msg;
            msg << "setTunerGain|Invalid tuner type. Must be RX_DIGITIZER or TX";
            LOG_ERROR(RFNoC_ProgrammableDevice_i,msg.str());
            throw FRONTEND::BadParameterException(msg.str().c_str());
        }
    } catch (std::exception& e) {
        std::ostringstream msg;
        msg << "setTunerGain|Exception: " << e.what();
        LOG_WARN(RFNoC_ProgrammableDevice_i,msg.str());
        throw FRONTEND::FrontendException(msg.str().c_str());
    }
}

float RFNoC_ProgrammableDevice_i::getTunerGain(const std::string& allocation_id)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].gain;
}

void RFNoC_ProgrammableDevice_i::setTunerReferenceSource(const std::string& allocation_id, long source)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);
    throw FRONTEND::NotSupportedException("setTunerReferenceSource not supported");
}

long RFNoC_ProgrammableDevice_i::getTunerReferenceSource(const std::string& allocation_id)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);
    throw FRONTEND::NotSupportedException("getTunerReferenceSource not supported");
}

void RFNoC_ProgrammableDevice_i::setTunerEnable(const std::string& allocation_id, bool enable)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);

    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    if(allocation_id != getControlAllocationId(idx)){
        std::ostringstream msg;
        msg << "setTunerEnable|ID (" << allocation_id << ") does not have authorization to modify tuner.";
        LOG_WARN(RFNoC_ProgrammableDevice_i,msg.str());
        throw FRONTEND::FrontendException(msg.str().c_str());
    }

    if (frontend_tuner_status[idx].tuner_type == "RX_DIGITIZER") {
        std::map<std::string, RxObject *>::iterator it = this->allocationIDToRx.find(allocation_id);

        if (it == this->allocationIDToRx) {
            std::ostringstream msg;
            msg << "setTunerEnable|ID (" << allocation_id << ") does not map to RX object.";
            LOG_ERROR(RFNoC_ProgrammableDevice_i,msg.str());
            throw FRONTEND::BadParameterException(msg.str().c_str());
        }

        this->radio->set_rx_streamer(enable, it->second->radioChannel);
    } else if (frontend_tuner_status[idx].tuner_type == "TX") {
        std::map<std::string, TxObject *>::iterator it = this->allocationIDToTx.find(allocation_id);

        if (it == this->allocationIDToTx) {
            std::ostringstream msg;
            msg << "setTunerEnable|ID (" << allocation_id << ") does not map to TX object.";
            LOG_ERROR(RFNoC_ProgrammableDevice_i,msg.str());
            throw FRONTEND::BadParameterException(msg.str().c_str());
        }

        this->radio->set_tx_streamer(enable, it->second->radioChannel);
    } else {
        std::ostringstream msg;
        msg << "setTunerEnable|Invalid tuner type. Must be RX_DIGITIZER or TX";
        LOG_ERROR(RFNoC_ProgrammableDevice_i,msg.str());
        throw FRONTEND::BadParameterException(msg.str().c_str());
    }

    frontend_tuner_status[idx].enabled = enable;
}

bool RFNoC_ProgrammableDevice_i::getTunerEnable(const std::string& allocation_id)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);

    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].enabled;
}

void RFNoC_ProgrammableDevice_i::setTunerOutputSampleRate(const std::string& allocation_id, double sr)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__ << " allocation_id=" << allocation_id << " sr=" << sr);

    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    if(allocation_id != getControlAllocationId(idx)){
        std::ostringstream msg;
        msg << "setTunerOutputSampleRate|ID (" << allocation_id << ") does not have authorization to modify tuner.";
        LOG_WARN(RFNoC_ProgrammableDevice_i,msg.str());
        throw FRONTEND::FrontendException(msg.str().c_str());
    }

    try {
        try {
            if (not frontend::validateRequest(125e3, 16e6, sr)) {
                std::ostringstream msg;
                msg << "setTunerOutputSampleRate|Invalid sample rate (" << sr <<")";
                LOG_WARN(RFNoC_ProgrammableDevice_i,msg.str() );
                throw FRONTEND::BadParameterException(msg.str().c_str());
            }
        } catch (FRONTEND::BadParameterException) {
            throw;
        } catch (...) {
            std::ostringstream msg;
            msg << "setTunerOutputSampleRate|Could not retrieve tuner_id to radio channel number mapping";
            LOG_ERROR(RFNoC_ProgrammableDevice_i,msg.str());
            throw FRONTEND::FrontendException(msg.str().c_str());
        }

        if (frontend_tuner_status[idx].tuner_type == "RX_DIGITIZER") {
            std::map<std::string, RxObject *>::iterator it = this->allocationIDToRx.find(allocation_id);

            if (it == this->allocationIDToRx) {
                std::ostringstream msg;
                msg << "setTunerOutputSampleRate|ID (" << allocation_id << ") does not map to RX object.";
                LOG_ERROR(RFNoC_ProgrammableDevice_i,msg.str());
                throw FRONTEND::BadParameterException(msg.str().c_str());
            }

            // If the freq has changed (change in stream) or the tuner is disabled, then set it as disabled
            bool is_tuner_enabled = frontend_tuner_status[idx].enabled;

            if (is_tuner_enabled) {
                it->second->ddc->set_rx_streamer(false, it->second->ddcPort);
            }

            uhd::rfnoc::ddc_block_ctrl::sptr ddc = it->second->ddc;
            size_t ddcPort = it->second->ddcPort;

            uhd::device_addr_t args;

            args["freq"] = "0.0";
            args["input_rate"] = boost::lexical_cast<std::string>(this->radio->get_output_samp_rate(it->second->radioChannel));
            args["output_rate"] = boost::lexical_cast<std::string>(sr);

            try {
                ddc->set_args(args, ddcPort);
            } catch(uhd::value_error &e) {
                std::ostringstream msg;
                msg << "setTunerOutputSampleRate|Exception: " << e.what();
                LOG_ERROR(RFNoC_ProgrammableDevice_i,msg.str());
                throw FRONTEND::BadParameterException(msg.str().c_str());
            } catch(...) {
                throw;
            }

            // Get the actual output rate of the DDC
            try {
                std::string sampleRateString;

                sampleRateString = ddc->get_arg("output_rate", ddcPort);

                frontend_tuner_status[idx].sample_rate = boost::lexical_cast<double>(sampleRateString);
            } catch(...) {
                throw;
            }

            // update status from hw
            it->second->updateSRI = true;

            // re-enable
            if (is_tuner_enabled) {
                it->second->ddc->set_rx_streamer(false, it->second->ddcPort);
            }
        } else if (frontend_tuner_status[idx].tuner_type == "TX") {
            std::map<std::string, TxObject *>::iterator it = this->allocationIDToTx.find(allocation_id);

            if (it == this->allocationIDToTx) {
                std::ostringstream msg;
                msg << "setTunerOutputSampleRate|ID (" << allocation_id << ") does not map to TX object.";
                LOG_ERROR(RFNoC_ProgrammableDevice_i,msg.str());
                throw FRONTEND::BadParameterException(msg.str().c_str());
            }

            // If the freq has changed (change in stream) or the tuner is disabled, then set it as disabled
            bool is_tuner_enabled = frontend_tuner_status[idx].enabled;

            if (is_tuner_enabled) {
                it->second->duc->set_tx_streamer(false, it->second->ducPort);
            }

            uhd::rfnoc::duc_block_ctrl::sptr duc = it->second->duc;
            size_t ducPort = it->second->ducPort;

            uhd::device_addr_t args;

            args["input_rate"] = boost::lexical_cast<std::string>(sr);
            args["output_rate"] = boost::lexical_cast<std::string>(this->radio->get_input_samp_rate(it->second->radioChannel));

            try {
                duc->set_args(args, ducPort);
            } catch(uhd::value_error &e) {
                std::ostringstream msg;
                msg << "setTunerOutputSampleRate|Exception: " << e.what();
                LOG_ERROR(RFNoC_ProgrammableDevice_i,msg.str());
                throw FRONTEND::BadParameterException(msg.str().c_str());
            } catch(...) {
                throw;
            }

            // Get the actual output rate of the DUC
            try {
                std::string sampleRateString;

                sampleRateString = duc->get_arg("input_rate", ducPort);

                frontend_tuner_status[idx].sample_rate = boost::lexical_cast<double>(sampleRateString);
            } catch(...) {
                throw;
            }

            // re-enable
            if (is_tuner_enabled) {
                it->second->duc->set_tx_streamer(false, it->second->ducPort);
            }
        } else {
            std::ostringstream msg;
            msg << "setTunerGain|Invalid tuner type. Must be RX_DIGITIZER or TX";
            LOG_ERROR(RFNoC_ProgrammableDevice_i,msg.str());
            throw FRONTEND::BadParameterException(msg.str().c_str());
        }
    } catch (std::exception& e) {
        std::ostringstream msg;
        msg << "setTunerGain|Exception: " << e.what();
        LOG_WARN(RFNoC_ProgrammableDevice_i,msg.str());
        throw FRONTEND::FrontendException(msg.str().c_str());
    }
}

double RFNoC_ProgrammableDevice_i::getTunerOutputSampleRate(const std::string& allocation_id)
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i,__PRETTY_FUNCTION__);

    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    LOG_DEBUG(RFNoC_ProgrammableDevice_i,"getTunerOutputSampleRate|TUNER_SR=" << frontend_tuner_status[idx].sample_rate);
    return frontend_tuner_status[idx].sample_rate;
}
