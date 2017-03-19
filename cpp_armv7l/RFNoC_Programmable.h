/*
 * RFNoCProgrammable.h
 *
 *  Created on: Mar 19, 2017
 *      Author: pwolfram
 */

#ifndef RFNOC_PROGRAMMABLE_H_
#define RFNOC_PROGRAMMABLE_H_

#include <omniORB4/omniORB.h>
#include <uhd/device3.hpp>

#include "RFNoC_Persona.h"
#include "RFNoC_Utils.h"

class RFNoC_Programmable {
    public:
        RFNoC_Programmable() {};
        virtual ~RFNoC_Programmable() {};

        virtual bool connectRadioRX(const CORBA::ULong &portHash, const BlockInfo &blockInfo) = 0;
        virtual bool connectRadioTX(const std::string &allocationId, const BlockInfo &blockInfo) = 0;
        virtual uhd::device3::sptr getUsrp() = 0;
        virtual void setGetBlockInfoFromHashCb(const std::string &resourceId, getBlockInfoFromHashCallback getBlockInfoFromHashCb) = 0;
};

#endif /* RFNOC_PROGRAMMABLE_H_ */
