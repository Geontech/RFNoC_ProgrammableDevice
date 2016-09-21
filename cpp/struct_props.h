#ifndef STRUCTPROPS_H
#define STRUCTPROPS_H

/*******************************************************************************************

    AUTO-GENERATED CODE. DO NOT MODIFY

*******************************************************************************************/

#include <ossie/CorbaUtils.h>
#include <CF/cf.h>
#include <ossie/PropertyMap.h>

struct hw_load_request_struct {
    hw_load_request_struct ()
    {
    };

    static std::string getId() {
        return std::string("hw_load_request");
    };

    std::string hardware_id;
    std::string load_filepath;
    std::string request_id;
    std::string requester_id;
};

inline bool operator>>= (const CORBA::Any& a, hw_load_request_struct& s) {
    CF::Properties* temp;
    if (!(a >>= temp)) return false;
    const redhawk::PropertyMap& props = redhawk::PropertyMap::cast(*temp);
    if (props.contains("hardware_id")) {
        if (!(props["hardware_id"] >>= s.hardware_id)) return false;
    }
    if (props.contains("load_filepath")) {
        if (!(props["load_filepath"] >>= s.load_filepath)) return false;
    }
    if (props.contains("request_id")) {
        if (!(props["request_id"] >>= s.request_id)) return false;
    }
    if (props.contains("requester_id")) {
        if (!(props["requester_id"] >>= s.requester_id)) return false;
    }
    return true;
}

inline void operator<<= (CORBA::Any& a, const hw_load_request_struct& s) {
    redhawk::PropertyMap props;
 
    props["hardware_id"] = s.hardware_id;
 
    props["load_filepath"] = s.load_filepath;
 
    props["request_id"] = s.request_id;
 
    props["requester_id"] = s.requester_id;
    a <<= props;
}

inline bool operator== (const hw_load_request_struct& s1, const hw_load_request_struct& s2) {
    if (s1.hardware_id!=s2.hardware_id)
        return false;
    if (s1.load_filepath!=s2.load_filepath)
        return false;
    if (s1.request_id!=s2.request_id)
        return false;
    if (s1.requester_id!=s2.requester_id)
        return false;
    return true;
}

inline bool operator!= (const hw_load_request_struct& s1, const hw_load_request_struct& s2) {
    return !(s1==s2);
}

struct hw_load_status_struct {
    hw_load_status_struct ()
    {
    };

    static std::string getId() {
        return std::string("hw_load_status");
    };

    std::string hardware_id;
    std::string load_filepath;
    std::string request_id;
    std::string requester_id;
    unsigned short state;
};

inline bool operator>>= (const CORBA::Any& a, hw_load_status_struct& s) {
    CF::Properties* temp;
    if (!(a >>= temp)) return false;
    const redhawk::PropertyMap& props = redhawk::PropertyMap::cast(*temp);
    if (props.contains("hw_load_status::hardware_id")) {
        if (!(props["hw_load_status::hardware_id"] >>= s.hardware_id)) return false;
    }
    if (props.contains("hw_load_status::load_filepath")) {
        if (!(props["hw_load_status::load_filepath"] >>= s.load_filepath)) return false;
    }
    if (props.contains("hw_load_status::request_id")) {
        if (!(props["hw_load_status::request_id"] >>= s.request_id)) return false;
    }
    if (props.contains("hw_load_status::requester_id")) {
        if (!(props["hw_load_status::requester_id"] >>= s.requester_id)) return false;
    }
    if (props.contains("hw_load_status::state")) {
        if (!(props["hw_load_status::state"] >>= s.state)) return false;
    }
    return true;
}

inline void operator<<= (CORBA::Any& a, const hw_load_status_struct& s) {
    redhawk::PropertyMap props;
 
    props["hw_load_status::hardware_id"] = s.hardware_id;
 
    props["hw_load_status::load_filepath"] = s.load_filepath;
 
    props["hw_load_status::request_id"] = s.request_id;
 
    props["hw_load_status::requester_id"] = s.requester_id;
 
    props["hw_load_status::state"] = s.state;
    a <<= props;
}

inline bool operator== (const hw_load_status_struct& s1, const hw_load_status_struct& s2) {
    if (s1.hardware_id!=s2.hardware_id)
        return false;
    if (s1.load_filepath!=s2.load_filepath)
        return false;
    if (s1.request_id!=s2.request_id)
        return false;
    if (s1.requester_id!=s2.requester_id)
        return false;
    if (s1.state!=s2.state)
        return false;
    return true;
}

inline bool operator!= (const hw_load_status_struct& s1, const hw_load_status_struct& s2) {
    return !(s1==s2);
}

#endif // STRUCTPROPS_H
