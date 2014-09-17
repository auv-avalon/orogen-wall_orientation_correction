#ifndef WALL_ESTIMATION_TASK_TYPES_HPP_
#define WALL_ESTIMATION_TASK_TYPES_HPP_

#include <vector>
#include <base/time.h>
#include <base/samples/pointcloud.h>
#include <sonar_detectors/WallAngleEstimationCandidate.hpp>

namespace sonar_detectors
{

struct WallEstimationDebugData
{
    base::Time time;
    base::samples::Pointcloud features;
    std::vector< WallCandidate > wall_candidates;
    WallEstimationDebugData()
    : time(base::Time::now()){}
};

}

#endif
