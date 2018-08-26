#ifndef RFNOC_PROGRAMMABLEDEVICE_ENTRY_POINTS_H
#define RFNOC_PROGRAMMABLEDEVICE_ENTRY_POINTS_H

// OSSIE Include(s)
#include <ossie/Device_impl.h>

// RF-NoC RH Include(s)
#include <RFNoC_Programmable.h>

typedef Device_impl* (*ConstructorPtr)(int, char*[], RFNoC_RH::RFNoC_Programmable *);

#endif
