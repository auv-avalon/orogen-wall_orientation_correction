/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace wall_orientation_correction;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

void Task::sonarbeam_featureTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &sonarbeam_feature_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d sonar2body;
    if (!_sonar2body.get(ts, sonar2body))
    {
        RTT::log(RTT::Error) << "skip, have no " << _sonar2body.getSourceFrame() << " to " << _sonar2body.getTargetFrame() << " transformation sample!" << RTT::endlog();
        new_state = MISSING_TRANSFORMATION;
        return;
    }
    
    Eigen::Affine3d body2odometry;
    if (!_body2odometry.get(ts, body2odometry))
    {
        RTT::log(RTT::Error) << "skip, have no " << _body2odometry.getSourceFrame() << " to " << _body2odometry.getTargetFrame() << " transformation sample!" << RTT::endlog();
        new_state = MISSING_TRANSFORMATION;
        return;
    }
    
    Eigen::Affine3d sonar2odometry = body2odometry * sonar2body;
    angle_estimation->updateFeature(sonarbeam_feature_sample, base::Angle::fromRad(base::getYaw(base::Orientation(sonar2odometry.linear()))));
    
    if(have_valid_wall_angle)
    {
	// receive wall to world transformation
	Eigen::Affine3d wall2world;
	if (!_wall2world.get(ts, wall2world))
	{
	    RTT::log(RTT::Error) << "skip, have no " << _wall2world.getSourceFrame() << " to " << _wall2world.getTargetFrame() << " transformation sample!" << RTT::endlog();
	    new_state = MISSING_TRANSFORMATION;
	    return;
	}
	
	Eigen::Affine3d body2wall = wall2odometry.inverse() * body2odometry;
	Eigen::Affine3d body2world = wall2world * body2wall;
	_angle_in_world.write(base::Angle::fromRad(base::getYaw(base::Orientation(body2world.linear()))));
    }
}

void Task::orientation_samplesTransformerCallback(const base::Time& ts, const base::samples::RigidBodyState& orientation_samples_sample)
{
    // receive wall to world transformation
    Eigen::Affine3d wall2world;
    if (!_wall2world.get(ts, wall2world))
    {
        RTT::log(RTT::Error) << "skip, have no " << _wall2world.getSourceFrame() << " to " << _wall2world.getTargetFrame() << " transformation sample!" << RTT::endlog();
        new_state = MISSING_TRANSFORMATION;
        return;
    }
    
    if(have_valid_wall_angle)
    {
	base::samples::RigidBodyState new_pose_sample = orientation_samples_sample;
	new_pose_sample.targetFrame = _target_frame.get();
	Eigen::Affine3d pose2wall = wall2odometry.inverse() * new_pose_sample.getTransform();
	new_pose_sample.setTransform(wall2world * pose2wall);
	_orientation_in_world.write(new_pose_sample);
    }
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    
    last_state = PRE_OPERATIONAL;
    new_state = RUNNING;
    wall2odometry = Eigen::Affine3d::Identity();
    have_valid_wall_angle = false;
    
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    have_valid_wall_angle = false;

    angle_estimation.reset(new sonar_detectors::WallAngleEstimation);
    angle_estimation->setRansacParameters(_ransac_max_distance.get(), _ransac_min_inlier.get());
    angle_estimation->setStabilityParameters(_wall_candidate_count.get(), _wall_angle_sigma.get(), _wall_distance_sigma.get());
    base::Angle left_limit = _initial_wall_direction.get() + _left_opening_angle.get();
    base::Angle right_limit = _initial_wall_direction.get() - _right_opening_angle.get();
    angle_estimation->setEstimationZone(left_limit, right_limit);
    
    return true;
}
void Task::updateHook()
{
    if(have_valid_wall_angle)
	new_state = VALID_WALL_FIX;
    else
	new_state = ESTIMATE_WALL_ORIENTATION;
    
    TaskBase::updateHook();

    base::Angle wall_angle;
    if(!have_valid_wall_angle && angle_estimation->getAngleToWall(wall_angle))
    {
	//set wall2odometry
	wall2odometry = Eigen::AngleAxisd(wall_angle.getRad(), Eigen::Vector3d::UnitZ());
	have_valid_wall_angle = true;
    }
    
    if(_enable_debug.value())
    {
	sonar_detectors::WallEstimationDebugData debug;
	debug.time = base::Time::now();
	debug.features.points = angle_estimation->getFeatures();
	debug.wall_candidates = angle_estimation->getCandidates();
	_debug_data.write(debug);
    }
    
    // write task state if it has changed
    if(last_state != new_state)
    {
        last_state = new_state;
        state(new_state);
    }
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
