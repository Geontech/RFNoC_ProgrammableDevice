#ifndef RFNOC_PERSONA_H
#define RFNOC_PERSONA_H

#include <boost/function.hpp>
#include <omniORB4/CORBA.h>

#include "RFNoC_Utils.h"

typedef boost::function<BlockInfo(const CORBA::ULong &portHash)> getBlockInfoFromHashCallback;

typedef boost::function<void(const std::string &deviceID, getBlockInfoFromHashCallback getBlockFromHashCb)> setGetBlockInfoFromHashCallback;

#endif
