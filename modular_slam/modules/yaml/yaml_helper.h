/**
 * @file yaml_helper.h
 * @author guyupan
 * @brief YAML processing helper utilities
 * @version 0.1
 * @date 2024-09-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

namespace modular_slam {

#define ENSURE_YAML_ENTRY_EXISTS(_c, _name) \
    ASSERTMSG_(                             \
      _c.has(_name),                        \
      mrpt::format(                         \
          "Missing YAML required entry, '%s'", std::string(_name).c_str()))

#define YAML_LOAD_MEMBER_OPT(_varname, _type) \
    _varname##_ = cfg.getOrDefault<_type>(#_varname, _varname##_)

#define YAML_LOAD_MEMBER_REQ(_varname, _type) \
    ENSURE_YAML_ENTRY_EXISTS(cfg, #_varname); \
    YAML_LOAD_MEMBER_OPT(_varname, _type)

}