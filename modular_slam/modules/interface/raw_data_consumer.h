/**
 * @file RawDataConsumer.h
 * @author guyupan
 * @brief virtual interface for raw data consumers, e.g SLAM front-ends
 * @version 0.1
 * @date 2024-09-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

#include <memory>

#include <mrpt/obs/CObservation.h>

namespace modular_slam {

class RawDataConsumer {
 public:
  RawDataConsumer()          = default;
  virtual ~RawDataConsumer() = default;

  virtual void OnNewObservation(const mrpt::obs::CObservation::Ptr& o) = 0;
};

}