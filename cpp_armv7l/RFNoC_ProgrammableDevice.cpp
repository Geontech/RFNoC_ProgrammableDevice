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
    HARDWARE_ID("E310")
{
}

RFNoC_ProgrammableDevice_i::RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev) :
    RFNoC_ProgrammableDevice_prog_base_type(devMgr_ior, id, lbl, sftwrPrfl, compDev),
    HARDWARE_ID("E310")
{
}

RFNoC_ProgrammableDevice_i::RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities) :
    RFNoC_ProgrammableDevice_prog_base_type(devMgr_ior, id, lbl, sftwrPrfl, capacities),
    HARDWARE_ID("E310")
{
}

RFNoC_ProgrammableDevice_i::RFNoC_ProgrammableDevice_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev) :
    RFNoC_ProgrammableDevice_prog_base_type(devMgr_ior, id, lbl, sftwrPrfl, capacities, compDev),
    HARDWARE_ID("E310")
{
}

RFNoC_ProgrammableDevice_i::~RFNoC_ProgrammableDevice_i()
{
}

void RFNoC_ProgrammableDevice_i::initialize() throw (CF::LifeCycle::InitializeError, CORBA::SystemException) 
{
    setHwLoadRequestsPtr(&hw_load_requests);
    setHwLoadStatusesPtr(&hw_load_statuses);

    // Attempt to get a reference to an e3x0 device
    uhd::device_addr_t addr;

    addr["type"] = "e3x0";

    this->usrp = uhd::usrp::multi_usrp::make(addr);

    if (not this->usrp->is_device3()) {
        LOG_FATAL(RFNoC_ProgrammableDevice_i, "Unable to find a suitable USRP Device 3.");
        throw CF::LifeCycle::InitializeError();
    }

    LOG_DEBUG(RFNoC_ProgrammableDevice_i, "Using Device: " << this->usrp->get_pp_string());

    // Allow some time for setup
    boost::this_thread::sleep(boost::posix_time::seconds(1.0));

    // Reset device streaming state
    this->usrp->get_device3()->clear();

    // Initialize the radios
    initializeRadios();

    // Clear the channels
    this->usrp->clear_channels();

    // Register the frontend callbacks
    this->setAllocationImpl(this->frontend_listener_allocation, this, &RFNoC_ProgrammableDevice_i::frontend_listener_allocation_alloc, &RFNoC_ProgrammableDevice_i::frontend_listener_allocation_dealloc);
    this->setAllocationImpl(this->frontend_tuner_allocation, this, &RFNoC_ProgrammableDevice_i::frontend_tuner_allocation_alloc, &RFNoC_ProgrammableDevice_i::frontend_tuner_allocation_dealloc);
}

/***********************************************************************************************

    Basic functionality:

        The service function is called by the serviceThread object (of type ProcessThread).
        This call happens immediately after the previous call if the return value for
        the previous call was NORMAL.
        If the return value for the previous call was NOOP, then the serviceThread waits
        an amount of time defined in the serviceThread's constructor.
        
    SRI:
        To create a StreamSRI object, use the following code:
                std::string stream_id = "testStream";
                BULKIO::StreamSRI sri = bulkio::sri::create(stream_id);

	Time:
	    To create a PrecisionUTCTime object, use the following code:
                BULKIO::PrecisionUTCTime tstamp = bulkio::time::utils::now();

        
    Ports:

        Data is passed to the serviceFunction through the getPacket call (BULKIO only).
        The dataTransfer class is a port-specific class, so each port implementing the
        BULKIO interface will have its own type-specific dataTransfer.

        The argument to the getPacket function is a floating point number that specifies
        the time to wait in seconds. A zero value is non-blocking. A negative value
        is blocking.  Constants have been defined for these values, bulkio::Const::BLOCKING and
        bulkio::Const::NON_BLOCKING.

        Each received dataTransfer is owned by serviceFunction and *MUST* be
        explicitly deallocated.

        To send data using a BULKIO interface, a convenience interface has been added 
        that takes a std::vector as the data input

        NOTE: If you have a BULKIO dataSDDS or dataVITA49 port, you must manually call 
              "port->updateStats()" to update the port statistics when appropriate.

        Example:
            // this example assumes that the device has two ports:
            //  A provides (input) port of type bulkio::InShortPort called short_in
            //  A uses (output) port of type bulkio::OutFloatPort called float_out
            // The mapping between the port and the class is found
            // in the device base class header file

            bulkio::InShortPort::dataTransfer *tmp = short_in->getPacket(bulkio::Const::BLOCKING);
            if (not tmp) { // No data is available
                return NOOP;
            }

            std::vector<float> outputData;
            outputData.resize(tmp->dataBuffer.size());
            for (unsigned int i=0; i<tmp->dataBuffer.size(); i++) {
                outputData[i] = (float)tmp->dataBuffer[i];
            }

            // NOTE: You must make at least one valid pushSRI call
            if (tmp->sriChanged) {
                float_out->pushSRI(tmp->SRI);
            }
            float_out->pushPacket(outputData, tmp->T, tmp->EOS, tmp->streamID);

            delete tmp; // IMPORTANT: MUST RELEASE THE RECEIVED DATA BLOCK
            return NORMAL;

        If working with complex data (i.e., the "mode" on the SRI is set to
        true), the std::vector passed from/to BulkIO can be typecast to/from
        std::vector< std::complex<dataType> >.  For example, for short data:

            bulkio::InShortPort::dataTransfer *tmp = myInput->getPacket(bulkio::Const::BLOCKING);
            std::vector<std::complex<short> >* intermediate = (std::vector<std::complex<short> >*) &(tmp->dataBuffer);
            // do work here
            std::vector<short>* output = (std::vector<short>*) intermediate;
            myOutput->pushPacket(*output, tmp->T, tmp->EOS, tmp->streamID);

        Interactions with non-BULKIO ports are left up to the device developer's discretion
        
    Messages:
    
        To receive a message, you need (1) an input port of type MessageEvent, (2) a message prototype described
        as a structure property of kind message, (3) a callback to service the message, and (4) to register the callback
        with the input port.
        
        Assuming a property of type message is declared called "my_msg", an input port called "msg_input" is declared of
        type MessageEvent, create the following code:
        
        void RFNoC_ProgrammableDevice_i::my_message_callback(const std::string& id, const my_msg_struct &msg){
        }
        
        Register the message callback onto the input port with the following form:
        this->msg_input->registerMessage("my_msg", this, &RFNoC_ProgrammableDevice_i::my_message_callback);
        
        To send a message, you need to (1) create a message structure, (2) a message prototype described
        as a structure property of kind message, and (3) send the message over the port.
        
        Assuming a property of type message is declared called "my_msg", an output port called "msg_output" is declared of
        type MessageEvent, create the following code:
        
        ::my_msg_struct msg_out;
        this->msg_output->sendMessage(msg_out);

    Properties:
        
        Properties are accessed directly as member variables. For example, if the
        property name is "baudRate", it may be accessed within member functions as
        "baudRate". Unnamed properties are given the property id as its name.
        Property types are mapped to the nearest C++ type, (e.g. "string" becomes
        "std::string"). All generated properties are declared in the base class
        (RFNoC_ProgrammableDevice_prog_base_type).
    
        Simple sequence properties are mapped to "std::vector" of the simple type.
        Struct properties, if used, are mapped to C++ structs defined in the
        generated file "struct_props.h". Field names are taken from the name in
        the properties file; if no name is given, a generated name of the form
        "field_n" is used, where "n" is the ordinal number of the field.
        
        Example:
            // This example makes use of the following Properties:
            //  - A float value called scaleValue
            //  - A boolean called scaleInput
              
            if (scaleInput) {
                dataOut[i] = dataIn[i] * scaleValue;
            } else {
                dataOut[i] = dataIn[i];
            }
            
        A callback method can be associated with a property so that the method is
        called each time the property value changes.  This is done by calling 
        setPropertyChangeListener(<property name>, this, &RFNoC_ProgrammableDevice_i::<callback method>)
        in the constructor.
            
        Example:
            // This example makes use of the following Properties:
            //  - A float value called scaleValue
            
        //Add to RFNoC_ProgrammableDevice.cpp
        RFNoC_ProgrammableDevice_i::RFNoC_ProgrammableDevice_i(const char *uuid, const char *label) :
            RFNoC_ProgrammableDevice_prog_base_type(uuid, label)
        {
            setPropertyChangeListener("scaleValue", this, &RFNoC_ProgrammableDevice_i::scaleChanged);
        }

        void RFNoC_ProgrammableDevice_i::scaleChanged(const std::string& id){
            std::cout << "scaleChanged scaleValue " << scaleValue << std::endl;
        }
            
        //Add to RFNoC_ProgrammableDevice.h
        void scaleChanged(const std::string&);
        
        
************************************************************************************************/
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
    // Generate the Persona Device
    Device_impl *persona = personaEntryPoint(argc, argv, this, boost::bind(&RFNoC_ProgrammableDevice_i::setHwLoadStatus, this, _1), this->usrp);

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

    uhd::device_addr_t addr;
    addr["type"] = "e3x0";

    uhd::image_loader::image_loader_args_t loader_args;
    loader_args.args = addr;
    loader_args.firmware_path = "";
    loader_args.fpga_path = requestStatus.load_filepath;
    loader_args.load_firmware = false;
    loader_args.load_fpga = true;

    if (not uhd::image_loader::load(loader_args)) {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Failed to load hardware.");
        return false;
    }

    return true;
}

void RFNoC_ProgrammableDevice_i::unloadHardware(const HwLoadStatusStruct& requestStatus) 
{
    // The hardware may be physically unloaded at this point
    LOG_INFO(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    uhd::device_addr_t addr;
    addr["type"] = "e3x0";

    uhd::image_loader::image_loader_args_t loader_args;
    loader_args.args = addr;
    loader_args.firmware_path = "";
    loader_args.fpga_path = "/usr/share/uhd/images/usrp_e310_fpga_idle.bit";
    loader_args.load_firmware = false;
    loader_args.load_fpga = true;

    if (not uhd::image_loader::load(loader_args)) {
        LOG_ERROR(RFNoC_ProgrammableDevice_i, "Failed to unload hardware.");
    }
}

bool RFNoC_ProgrammableDevice_i::hwLoadRequestIsValid(const HwLoadRequestStruct& hwLoadRequestStruct)
{
    if (hwLoadRequestStruct.hardware_id != this->HARDWARE_ID) {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to validate hardware load request. Mismatched hardware IDs.");
        return false;
    }

    if (not boost::filesystem::exists(hwLoadRequestStruct.load_filepath)) {
        LOG_WARN(RFNoC_ProgrammableDevice_i, "Failed to validate hardware load request. Load file path is invalid.");
        return false;
    }

    return true;
}

void RFNoC_ProgrammableDevice_i::initializeRadios()
{
    std::vector<std::string> nocBlocks = listNoCBlocks();

    for (size_t i = 0; i < nocBlocks.size(); ++i) {
        if (nocBlocks[i].find("Radio") != std::string::npos) {
            LOG_INFO(RFNoC_ProgrammableDevice_i, "Found a radio: " << nocBlocks[i]);

            this->radioIDs.push_back(nocBlocks[i]);
        }
    }

    setNumChannels(this->usrp->get_rx_num_channels() + this->usrp->get_tx_num_channels());

    size_t i;

    for (i = 0; i < this->usrp->get_rx_num_channels(); ++i) {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "Adding an RX channel");
        this->rxStatuses.push_back(&this->frontend_tuner_status[i]);
    }

    for (; i < this->frontend_tuner_status.size(); ++i) {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "Adding a TX channel");
        this->frontend_tuner_status[i].tuner_type = "TX";

        this->txStatuses.push_back(&this->frontend_tuner_status[i]);
    }

    LOG_INFO(RFNoC_ProgrammableDevice_i, "There are " << this->rxStatuses.size() << " RX channels");

    for (size_t i = 0; i < this->rxStatuses.size(); ++i) {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "Gathering info for RX Channel " << i);
        frontend_tuner_status_struct_struct &fts = *this->rxStatuses[i];

        double bw = this->usrp->get_rx_bandwidth(i);

        fts.bandwidth = bw;

        uhd::freq_range_t bwRange = this->usrp->get_rx_bandwidth_range(i);

        std::stringstream ss;

        for (size_t j = 0; j < bwRange.size(); ++j) {
            if (j != 0) {
                ss << ",";
            }

            ss << std::fixed << bwRange[j].start() << "-" << bwRange.stop();
        }

        fts.available_bandwidth = ss.str();

        ss.str(std::string());

        double cf = this->usrp->get_rx_freq(i);

        fts.center_frequency = cf;

        uhd::freq_range_t cfRange = this->usrp->get_rx_freq_range(i);

        for (size_t j = 0; j < cfRange.size(); ++j) {
            if (j != 0) {
                ss << ",";
            }

            ss << std::fixed << cfRange[j].start() << "-" << cfRange[j].stop();
        }

        fts.available_frequency = ss.str();

        ss.str(std::string());

        double sr = this->usrp->get_rx_rate(i);

        fts.sample_rate = sr;

        uhd::meta_range_t srRange = this->usrp->get_rx_rates(i);

        for (size_t j = 0; j < srRange.size(); ++j) {
            if (j != 0) {
                ss << ",";
            }

            ss << std::fixed << srRange[j].start() << "-" << srRange[j].stop();
        }

        fts.available_sample_rate = ss.str();
    }

    for (size_t i = 0; i < this->txStatuses.size(); ++i) {
        LOG_INFO(RFNoC_ProgrammableDevice_i, "Gathering info for TX Channel " << i);
        frontend_tuner_status_struct_struct &fts = *this->txStatuses[i];

        double bw = this->usrp->get_tx_bandwidth(i);

        fts.bandwidth = bw;

        uhd::freq_range_t bwRange = this->usrp->get_tx_bandwidth_range(i);

        std::stringstream ss;

        for (size_t j = 0; j < bwRange.size(); ++j) {
            if (j != 0) {
                ss << ",";
            }

            ss << std::fixed << bwRange[j].start() << "-" << bwRange.stop();
        }

        fts.available_bandwidth = ss.str();

        ss.str(std::string());

        double cf = this->usrp->get_tx_freq(i);

        fts.center_frequency = cf;

        uhd::freq_range_t cfRange = this->usrp->get_tx_freq_range(i);

        for (size_t j = 0; j < cfRange.size(); ++j) {
            if (j != 0) {
                ss << ",";
            }

            ss << std::fixed << cfRange[j].start() << "-" << cfRange[j].stop();
        }

        fts.available_frequency = ss.str();

        ss.str(std::string());

        double sr = this->usrp->get_tx_rate(i);

        fts.sample_rate = sr;

        uhd::meta_range_t srRange = this->usrp->get_tx_rates(i);

        for (size_t j = 0; j < srRange.size(); ++j) {
            if (j != 0) {
                ss << ",";
            }

            ss << std::fixed << srRange[j].start() << "-" << srRange[j].stop();
        }

        fts.available_sample_rate = ss.str();
    }
}

std::vector<std::string> RFNoC_ProgrammableDevice_i::listNoCBlocks()
{
    LOG_TRACE(RFNoC_ProgrammableDevice_i, __PRETTY_FUNCTION__);

    std::vector<std::string> NoCBlocks;

    uhd::property_tree::sptr tree = this->usrp->get_device3()->get_tree();

    std::vector<std::string> xBarItems = tree->list("/mboards/0/xbar/");

    for (size_t i = 0; i < xBarItems.size(); ++i) {
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, xBarItems[i]);

        NoCBlocks.push_back("0/" + xBarItems[i]);
    }

    return NoCBlocks;
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

bool RFNoC_ProgrammableDevice_i::frontend_listener_allocation_alloc(const frontend_listener_allocation_struct &newAllocation)
{
    // Check validity of allocation_id's
    if (frontend_listener_allocation.existing_allocation_id.empty()){
        LOG_INFO(RFNoC_ProgrammableDevice_i,"allocateCapacity: MISSING EXISTING ALLOCATION ID");
        return false;
    }
    if (frontend_listener_allocation.listener_allocation_id.empty()){
        LOG_INFO(RFNoC_ProgrammableDevice_i,"allocateCapacity: MISSING LISTENER ALLOCATION ID");
        return false;
    }

    exclusive_lock lock(allocation_id_mapping_lock);

    // Check if listener allocation ID has already been used
    if(getTunerMapping(frontend_listener_allocation.listener_allocation_id) >= 0){
        LOG_INFO(RFNoC_ProgrammableDevice_i,"allocateCapacity: LISTENER ALLOCATION ID ALREADY IN USE: [" << frontend_listener_allocation.listener_allocation_id << "]");
        return false;
    }
    // Do not allocate if existing allocation ID does not exist
    long tuner_id = getTunerMapping(frontend_listener_allocation.existing_allocation_id);
    if (tuner_id < 0){
        LOG_DEBUG(RFNoC_ProgrammableDevice_i,"allocateCapacity: UNKNOWN CONTROL ALLOCATION ID: ["<< frontend_listener_allocation.existing_allocation_id <<"]");
        return false;
    }

    // listener allocations are not permitted for channelizers or TX
    if(frontend_tuner_status[tuner_id].tuner_type == "CHANNELIZER" || frontend_tuner_status[tuner_id].tuner_type == "TX"){
        std::ostringstream eout;
        eout<<"allocateCapacity: listener allocations are not permitted for " << std::string(frontend_tuner_status[tuner_id].tuner_type) << " tuner type";
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, eout.str());
        return false;
    }

    tuner_allocation_ids[tuner_id].listener_allocation_ids.push_back(frontend_listener_allocation.listener_allocation_id);
    allocation_id_to_tuner_id.insert(std::pair<std::string, size_t > (frontend_listener_allocation.listener_allocation_id, tuner_id));
    frontend_tuner_status[tuner_id].allocation_id_csv = createAllocationIdCsv(tuner_id);
    this->assignListener(frontend_listener_allocation.listener_allocation_id,frontend_listener_allocation.existing_allocation_id);

    return true;
}

void RFNoC_ProgrammableDevice_i::frontend_listener_allocation_dealloc(const frontend_listener_allocation_struct &newDeallocation)
{
    long tuner_id = getTunerMapping(frontend_listener_allocation.listener_allocation_id);
    if (tuner_id < 0){
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "ALLOCATION_ID NOT FOUND: [" << frontend_listener_allocation.listener_allocation_id <<"]");
        return;
    }

    // send EOS to listener connection only
    removeTunerMapping(tuner_id,frontend_listener_allocation.listener_allocation_id);
    frontend_tuner_status[tuner_id].allocation_id_csv = createAllocationIdCsv(tuner_id);
}

bool RFNoC_ProgrammableDevice_i::frontend_tuner_allocation_alloc(const frontend_tuner_allocation_struct &newAllocation)
{
    // Check allocation_id
    if (frontend_tuner_allocation.allocation_id.empty()) {
        LOG_INFO(RFNoC_ProgrammableDevice_i,"allocateCapacity: MISSING ALLOCATION_ID");
        return false;
    }
    // Check if allocation ID has already been used
    if(getTunerMapping(frontend_tuner_allocation.allocation_id) >= 0){
        LOG_INFO(RFNoC_ProgrammableDevice_i,"allocateCapacity: ALLOCATION_ID ALREADY IN USE: [" << frontend_tuner_allocation.allocation_id << "]");
        return false;
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
                return false;
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
}

void RFNoC_ProgrammableDevice_i::frontend_tuner_allocation_dealloc(const frontend_tuner_allocation_struct &newDeallocation)
{
    // Try to remove control of the device
    long tuner_id = getTunerMapping(frontend_tuner_allocation.allocation_id);
    if (tuner_id < 0){
        LOG_DEBUG(RFNoC_ProgrammableDevice_i, "ALLOCATION_ID NOT FOUND: [" << frontend_tuner_allocation.allocation_id <<"]");
        return;
    }

    if(tuner_allocation_ids[tuner_id].control_allocation_id == frontend_tuner_allocation.allocation_id){
        enableTuner(tuner_id, false);
        removeTunerMapping(tuner_id);
        frontend_tuner_status[tuner_id].allocation_id_csv = createAllocationIdCsv(tuner_id);
    }
    else {
        // send EOS to listener connection only
        removeTunerMapping(tuner_id,frontend_tuner_allocation.allocation_id);
        frontend_tuner_status[tuner_id].allocation_id_csv = createAllocationIdCsv(tuner_id);
    }
}

