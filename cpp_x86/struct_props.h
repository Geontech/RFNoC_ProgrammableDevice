#ifndef STRUCTPROPS_H
#define STRUCTPROPS_H

/*******************************************************************************************

    AUTO-GENERATED CODE. DO NOT MODIFY

*******************************************************************************************/

#include <ossie/CorbaUtils.h>
#include <CF/cf.h>
#include <ossie/PropertyMap.h>
#include <bulkio/bulkio.h>
typedef bulkio::connection_descriptor_struct connection_descriptor_struct;

struct frontend_listener_allocation_struct {
    frontend_listener_allocation_struct ()
    {
    };

    static std::string getId() {
        return std::string("FRONTEND::listener_allocation");
    };

    std::string existing_allocation_id;
    std::string listener_allocation_id;
};

inline bool operator>>= (const CORBA::Any& a, frontend_listener_allocation_struct& s) {
    CF::Properties* temp;
    if (!(a >>= temp)) return false;
    const redhawk::PropertyMap& props = redhawk::PropertyMap::cast(*temp);
    if (props.contains("FRONTEND::listener_allocation::existing_allocation_id")) {
        if (!(props["FRONTEND::listener_allocation::existing_allocation_id"] >>= s.existing_allocation_id)) return false;
    }
    if (props.contains("FRONTEND::listener_allocation::listener_allocation_id")) {
        if (!(props["FRONTEND::listener_allocation::listener_allocation_id"] >>= s.listener_allocation_id)) return false;
    }
    return true;
}

inline void operator<<= (CORBA::Any& a, const frontend_listener_allocation_struct& s) {
    redhawk::PropertyMap props;
 
    props["FRONTEND::listener_allocation::existing_allocation_id"] = s.existing_allocation_id;
 
    props["FRONTEND::listener_allocation::listener_allocation_id"] = s.listener_allocation_id;
    a <<= props;
}

inline bool operator== (const frontend_listener_allocation_struct& s1, const frontend_listener_allocation_struct& s2) {
    if (s1.existing_allocation_id!=s2.existing_allocation_id)
        return false;
    if (s1.listener_allocation_id!=s2.listener_allocation_id)
        return false;
    return true;
}

inline bool operator!= (const frontend_listener_allocation_struct& s1, const frontend_listener_allocation_struct& s2) {
    return !(s1==s2);
}

struct frontend_tuner_allocation_struct {
    frontend_tuner_allocation_struct ()
    {
        center_frequency = 0.0;
        bandwidth = 0.0;
        bandwidth_tolerance = 10.0;
        sample_rate = 0.0;
        sample_rate_tolerance = 10.0;
        device_control = true;
    };

    static std::string getId() {
        return std::string("FRONTEND::tuner_allocation");
    };

    std::string tuner_type;
    std::string allocation_id;
    double center_frequency;
    double bandwidth;
    double bandwidth_tolerance;
    double sample_rate;
    double sample_rate_tolerance;
    bool device_control;
    std::string group_id;
    std::string rf_flow_id;
};

inline bool operator>>= (const CORBA::Any& a, frontend_tuner_allocation_struct& s) {
    CF::Properties* temp;
    if (!(a >>= temp)) return false;
    const redhawk::PropertyMap& props = redhawk::PropertyMap::cast(*temp);
    if (props.contains("FRONTEND::tuner_allocation::tuner_type")) {
        if (!(props["FRONTEND::tuner_allocation::tuner_type"] >>= s.tuner_type)) return false;
    }
    if (props.contains("FRONTEND::tuner_allocation::allocation_id")) {
        if (!(props["FRONTEND::tuner_allocation::allocation_id"] >>= s.allocation_id)) return false;
    }
    if (props.contains("FRONTEND::tuner_allocation::center_frequency")) {
        if (!(props["FRONTEND::tuner_allocation::center_frequency"] >>= s.center_frequency)) return false;
    }
    if (props.contains("FRONTEND::tuner_allocation::bandwidth")) {
        if (!(props["FRONTEND::tuner_allocation::bandwidth"] >>= s.bandwidth)) return false;
    }
    if (props.contains("FRONTEND::tuner_allocation::bandwidth_tolerance")) {
        if (!(props["FRONTEND::tuner_allocation::bandwidth_tolerance"] >>= s.bandwidth_tolerance)) return false;
    }
    if (props.contains("FRONTEND::tuner_allocation::sample_rate")) {
        if (!(props["FRONTEND::tuner_allocation::sample_rate"] >>= s.sample_rate)) return false;
    }
    if (props.contains("FRONTEND::tuner_allocation::sample_rate_tolerance")) {
        if (!(props["FRONTEND::tuner_allocation::sample_rate_tolerance"] >>= s.sample_rate_tolerance)) return false;
    }
    if (props.contains("FRONTEND::tuner_allocation::device_control")) {
        if (!(props["FRONTEND::tuner_allocation::device_control"] >>= s.device_control)) return false;
    }
    if (props.contains("FRONTEND::tuner_allocation::group_id")) {
        if (!(props["FRONTEND::tuner_allocation::group_id"] >>= s.group_id)) return false;
    }
    if (props.contains("FRONTEND::tuner_allocation::rf_flow_id")) {
        if (!(props["FRONTEND::tuner_allocation::rf_flow_id"] >>= s.rf_flow_id)) return false;
    }
    return true;
}

inline void operator<<= (CORBA::Any& a, const frontend_tuner_allocation_struct& s) {
    redhawk::PropertyMap props;
 
    props["FRONTEND::tuner_allocation::tuner_type"] = s.tuner_type;
 
    props["FRONTEND::tuner_allocation::allocation_id"] = s.allocation_id;
 
    props["FRONTEND::tuner_allocation::center_frequency"] = s.center_frequency;
 
    props["FRONTEND::tuner_allocation::bandwidth"] = s.bandwidth;
 
    props["FRONTEND::tuner_allocation::bandwidth_tolerance"] = s.bandwidth_tolerance;
 
    props["FRONTEND::tuner_allocation::sample_rate"] = s.sample_rate;
 
    props["FRONTEND::tuner_allocation::sample_rate_tolerance"] = s.sample_rate_tolerance;
 
    props["FRONTEND::tuner_allocation::device_control"] = s.device_control;
 
    props["FRONTEND::tuner_allocation::group_id"] = s.group_id;
 
    props["FRONTEND::tuner_allocation::rf_flow_id"] = s.rf_flow_id;
    a <<= props;
}

inline bool operator== (const frontend_tuner_allocation_struct& s1, const frontend_tuner_allocation_struct& s2) {
    if (s1.tuner_type!=s2.tuner_type)
        return false;
    if (s1.allocation_id!=s2.allocation_id)
        return false;
    if (s1.center_frequency!=s2.center_frequency)
        return false;
    if (s1.bandwidth!=s2.bandwidth)
        return false;
    if (s1.bandwidth_tolerance!=s2.bandwidth_tolerance)
        return false;
    if (s1.sample_rate!=s2.sample_rate)
        return false;
    if (s1.sample_rate_tolerance!=s2.sample_rate_tolerance)
        return false;
    if (s1.device_control!=s2.device_control)
        return false;
    if (s1.group_id!=s2.group_id)
        return false;
    if (s1.rf_flow_id!=s2.rf_flow_id)
        return false;
    return true;
}

inline bool operator!= (const frontend_tuner_allocation_struct& s1, const frontend_tuner_allocation_struct& s2) {
    return !(s1==s2);
}

struct hw_load_request_struct_struct {
    hw_load_request_struct_struct ()
    {
    };

    static std::string getId() {
        return std::string("hw_load_request_struct");
    };

    std::string hardware_id;
    std::string load_filepath;
    std::string request_id;
    std::string requester_id;
};

inline bool operator>>= (const CORBA::Any& a, hw_load_request_struct_struct& s) {
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

inline void operator<<= (CORBA::Any& a, const hw_load_request_struct_struct& s) {
    redhawk::PropertyMap props;
 
    props["hardware_id"] = s.hardware_id;
 
    props["load_filepath"] = s.load_filepath;
 
    props["request_id"] = s.request_id;
 
    props["requester_id"] = s.requester_id;
    a <<= props;
}

inline bool operator== (const hw_load_request_struct_struct& s1, const hw_load_request_struct_struct& s2) {
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

inline bool operator!= (const hw_load_request_struct_struct& s1, const hw_load_request_struct_struct& s2) {
    return !(s1==s2);
}

struct hw_load_statuses_struct_struct {
    hw_load_statuses_struct_struct ()
    {
    };

    static std::string getId() {
        return std::string("hw_load_statuses_struct");
    };

    std::string hardware_id;
    std::string load_filepath;
    std::string request_id;
    std::string requester_id;
    unsigned short state;
};

inline bool operator>>= (const CORBA::Any& a, hw_load_statuses_struct_struct& s) {
    CF::Properties* temp;
    if (!(a >>= temp)) return false;
    const redhawk::PropertyMap& props = redhawk::PropertyMap::cast(*temp);
    if (props.contains("hw_load_statuses::hardware_id")) {
        if (!(props["hw_load_statuses::hardware_id"] >>= s.hardware_id)) return false;
    }
    if (props.contains("hw_load_statuses::load_filepath")) {
        if (!(props["hw_load_statuses::load_filepath"] >>= s.load_filepath)) return false;
    }
    if (props.contains("hw_load_statuses::request_id")) {
        if (!(props["hw_load_statuses::request_id"] >>= s.request_id)) return false;
    }
    if (props.contains("hw_load_statuses::requester_id")) {
        if (!(props["hw_load_statuses::requester_id"] >>= s.requester_id)) return false;
    }
    if (props.contains("hw_load_statuses::state")) {
        if (!(props["hw_load_statuses::state"] >>= s.state)) return false;
    }
    return true;
}

inline void operator<<= (CORBA::Any& a, const hw_load_statuses_struct_struct& s) {
    redhawk::PropertyMap props;
 
    props["hw_load_statuses::hardware_id"] = s.hardware_id;
 
    props["hw_load_statuses::load_filepath"] = s.load_filepath;
 
    props["hw_load_statuses::request_id"] = s.request_id;
 
    props["hw_load_statuses::requester_id"] = s.requester_id;
 
    props["hw_load_statuses::state"] = s.state;
    a <<= props;
}

inline bool operator== (const hw_load_statuses_struct_struct& s1, const hw_load_statuses_struct_struct& s2) {
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

inline bool operator!= (const hw_load_statuses_struct_struct& s1, const hw_load_statuses_struct_struct& s2) {
    return !(s1==s2);
}

struct frontend_tuner_status_struct_struct {
    frontend_tuner_status_struct_struct ()
    {
    };

    static std::string getId() {
        return std::string("FRONTEND::tuner_status_struct");
    };

    std::string allocation_id_csv;
    std::string available_bandwidth;
    std::string available_frequency;
    std::string available_sample_rate;
    double bandwidth;
    double center_frequency;
    bool enabled;
    std::string group_id;
    std::string rf_flow_id;
    double sample_rate;
    std::string stream_id;
    std::string tuner_type;
};

inline bool operator>>= (const CORBA::Any& a, frontend_tuner_status_struct_struct& s) {
    CF::Properties* temp;
    if (!(a >>= temp)) return false;
    const redhawk::PropertyMap& props = redhawk::PropertyMap::cast(*temp);
    if (props.contains("FRONTEND::tuner_status::allocation_id_csv")) {
        if (!(props["FRONTEND::tuner_status::allocation_id_csv"] >>= s.allocation_id_csv)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::available_bandwidth")) {
        if (!(props["FRONTEND::tuner_status::available_bandwidth"] >>= s.available_bandwidth)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::available_frequency")) {
        if (!(props["FRONTEND::tuner_status::available_frequency"] >>= s.available_frequency)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::available_sample_rate")) {
        if (!(props["FRONTEND::tuner_status::available_sample_rate"] >>= s.available_sample_rate)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::bandwidth")) {
        if (!(props["FRONTEND::tuner_status::bandwidth"] >>= s.bandwidth)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::center_frequency")) {
        if (!(props["FRONTEND::tuner_status::center_frequency"] >>= s.center_frequency)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::enabled")) {
        if (!(props["FRONTEND::tuner_status::enabled"] >>= s.enabled)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::group_id")) {
        if (!(props["FRONTEND::tuner_status::group_id"] >>= s.group_id)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::rf_flow_id")) {
        if (!(props["FRONTEND::tuner_status::rf_flow_id"] >>= s.rf_flow_id)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::sample_rate")) {
        if (!(props["FRONTEND::tuner_status::sample_rate"] >>= s.sample_rate)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::stream_id")) {
        if (!(props["FRONTEND::tuner_status::stream_id"] >>= s.stream_id)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::tuner_type")) {
        if (!(props["FRONTEND::tuner_status::tuner_type"] >>= s.tuner_type)) return false;
    }
    return true;
}

inline void operator<<= (CORBA::Any& a, const frontend_tuner_status_struct_struct& s) {
    redhawk::PropertyMap props;
 
    props["FRONTEND::tuner_status::allocation_id_csv"] = s.allocation_id_csv;
 
    props["FRONTEND::tuner_status::available_bandwidth"] = s.available_bandwidth;
 
    props["FRONTEND::tuner_status::available_frequency"] = s.available_frequency;
 
    props["FRONTEND::tuner_status::available_sample_rate"] = s.available_sample_rate;
 
    props["FRONTEND::tuner_status::bandwidth"] = s.bandwidth;
 
    props["FRONTEND::tuner_status::center_frequency"] = s.center_frequency;
 
    props["FRONTEND::tuner_status::enabled"] = s.enabled;
 
    props["FRONTEND::tuner_status::group_id"] = s.group_id;
 
    props["FRONTEND::tuner_status::rf_flow_id"] = s.rf_flow_id;
 
    props["FRONTEND::tuner_status::sample_rate"] = s.sample_rate;
 
    props["FRONTEND::tuner_status::stream_id"] = s.stream_id;
 
    props["FRONTEND::tuner_status::tuner_type"] = s.tuner_type;
    a <<= props;
}

inline bool operator== (const frontend_tuner_status_struct_struct& s1, const frontend_tuner_status_struct_struct& s2) {
    if (s1.allocation_id_csv!=s2.allocation_id_csv)
        return false;
    if (s1.available_bandwidth!=s2.available_bandwidth)
        return false;
    if (s1.available_frequency!=s2.available_frequency)
        return false;
    if (s1.available_sample_rate!=s2.available_sample_rate)
        return false;
    if (s1.bandwidth!=s2.bandwidth)
        return false;
    if (s1.center_frequency!=s2.center_frequency)
        return false;
    if (s1.enabled!=s2.enabled)
        return false;
    if (s1.group_id!=s2.group_id)
        return false;
    if (s1.rf_flow_id!=s2.rf_flow_id)
        return false;
    if (s1.sample_rate!=s2.sample_rate)
        return false;
    if (s1.stream_id!=s2.stream_id)
        return false;
    if (s1.tuner_type!=s2.tuner_type)
        return false;
    return true;
}

inline bool operator!= (const frontend_tuner_status_struct_struct& s1, const frontend_tuner_status_struct_struct& s2) {
    return !(s1==s2);
}

#endif // STRUCTPROPS_H
