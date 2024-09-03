/**
 * @file RawDataSourceBase.cc
 * @author guyupan
 * @brief virtual interface for datasets
 * @version 0.1
 * @date 2024-09-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "modules/interface/raw_datasource_base.h"

namespace modular_slam {

RawDataSourceBase::RawDataSourceBase() = default;

RawDataSourceBase::~RawDataSourceBase() = default;

void RawDataSourceBase::AttachToDataConsumer(RawDataConsumer& rdc) {
  rdc_.push_back(&rdc);
}

void RawDataSourceBase::SendObservationsToFrontEnds(
  const mrpt::obs::CObservation::Ptr& obs) {
  
  if (!rdc_.empty()) {
    for (auto& subscriber : rdc_) {
      subscriber->OnNewObservation(obs);
    }
  }

}

}