#ifndef HWLOADSTATUS_H
#define HWLOADSTATUS_H

#include <boost/bind.hpp>
#include <boost/function.hpp>

struct hw_load_status_object {
    std::string hardware_id;
    std::string load_filepath;
    std::string request_id;
    std::string requester_id;
    unsigned short state;
};

typedef boost::function<void(const hw_load_status_object &hwLoadStatus)> hwLoadStatusCallback;

#endif
