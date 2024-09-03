/**
 * @file RawDataSourceBase.h
 * @author guyupan
 * @brief virtual interface for datasets
 * @version 0.1
 * @date 2024-08-30
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

#include <vector>

#include "mrpt/containers/yaml.h"

#include "modules/interface/raw_data_consumer.h"

namespace modular_slam {

class RawDataSourceBase {
 public:
  RawDataSourceBase();
  virtual ~RawDataSourceBase();
  
  void AttachToDataConsumer(RawDataConsumer& rdc);

  virtual void Initialize_rds(const mrpt::containers::yaml& yaml) = 0;

 protected:
  void SendObservationsToFrontEnds(const mrpt::obs::CObservation::Ptr& obs);

 private:
 /** Target of captured data */
  std::vector<RawDataConsumer*> rdc_;

};

}