/**
 * @file OfflineDatasetSource.h
 * @author guyupan
 * @brief virtual interface for offline dataset sources
 * @version 0.1
 * @date 2024-09-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

#include "mrpt/obs/CSensoryFrame.h"
#include "mrpt/poses/CPose3DInterpolator.h"

namespace modular_slam {

class OfflineDatasetSource {
 public:
  OfflineDatasetSource()          = default;
  virtual ~OfflineDatasetSource() = default;

  virtual size_t DatasetSize() const = 0;

  virtual mrpt::obs::CSensoryFrame::Ptr GetObservations(size_t timestamp) const = 0;
 
  virtual bool HasGroundTruthTrajectory() const { return false; }

  virtual mrpt::poses::CPose3DInterpolator GetGroundTruthTrajectory() const { return {}; }

};

}