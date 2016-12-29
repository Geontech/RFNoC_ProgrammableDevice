#ifndef RFNOC_PERSONA_H
#define RFNOC_PERSONA_H

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <omniORB4/CORBA.h>
#include <uhd/device3.hpp>
#include <uhd/rfnoc/block_id.hpp>

#include "RFNoC_Utils.h"

typedef boost::function<bool(const std::string &allocationID, const uhd::rfnoc::block_id_t &blockToConnect, const size_t &blockPort)> connectRadioTXCallback;

typedef boost::function<BlockInfo(const CORBA::ULong &portHash)> getBlockInfoFromHashCallback;

typedef boost::function<uhd::device3::sptr()> getUsrpCallback;

struct hw_load_status_object {
    std::string hardware_id;
    std::string load_filepath;
    std::string request_id;
    std::string requester_id;
    unsigned short state;
};

typedef boost::function<void(const std::string &deviceID, const hw_load_status_object &hwLoadStatus)> hwLoadStatusCallback;

typedef boost::function<void(const std::string &deviceID, getBlockInfoFromHashCallback getBlockFromHashCb)> setGetBlockInfoFromHashCallback;

#endif
