/*
 * This file is part of Klampt_grasp_simulator.
 * Copyright (C) 2021  Marc Riedlinger

 * Klampt_grasp_simulator is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

// Abstract base class of all gripper interfaces which has to be subclassed


#include <Klampt_grasp_simulator/gripper_interface_base.h>
#include <KrisLibrary/robotics/IK.h>
#include <KrisLibrary/robotics/IKFunctions.h>
#include <Klampt_grasp_simulator/calculation_utilities.h>

// All gripper interfaces headers
#include <Klampt_grasp_simulator/gripper_interface_cob4_old.h>


#define MAX_ALLOWED_FORCE 100.0  // max allowed force per link, used for stuck detection


GripperInterfaceBase* GripperInterfaceBase::createGripperInterfaceByName(const std::string &interface_name, ros::NodeHandle* nh, RobotWorld &world, WorldSimulation &simulation)
{
	if ( interface_name.compare("cob4_old") == 0 )
		return new GripperInterfaceCob4Old(nh, world, simulation);
	// else if ( interface_name.compare("my_gripper") == 0 )  // add further interfaces here
	else
		ROS_ERROR("GripperInterfaceBase::createInterfaceByName - Could not find a matching interface for interface name %s!", interface_name.c_str());

	return nullptr;
}

GripperInterfaceBase::GripperInterfaceBase(ros::NodeHandle* nh, RobotWorld &world, WorldSimulation &simulation) :
					world_(world), simulation_(simulation), controller_(nullptr), controlled_simulator_(nullptr), global_gripper_movement_timeout_(0), initialized_(false),
					gripper_alignment_vector_(0,0,0)
{
	nh->param<int>("gripper_index", gripper_index_, 0);

	if ( gripper_index_ >= 0 && gripper_index_ < world_.robots.size() )
		gripper_ = world_.robots[gripper_index_];  // store robot for faster access

	if ( gripper_ == nullptr )
	{
		ROS_ERROR("GripperInterfaceBase::GripperInterfaceBase - gripper_index %d is out of bounds.", gripper_index_);
		return;
	}

	if ( gripper_index_ < simulation_.robotControllers.size() )
		controller_ = simulation_.robotControllers[gripper_index_];

	if ( controller_ == nullptr )
	{
		ROS_ERROR("GripperInterfaceBase::GripperInterfaceBase - No robot controller available for gripper at index %d", gripper_index_);
		return;
	}

	if ( gripper_index_ < simulation_.controlSimulators.size() )
		controlled_simulator_ = &simulation_.controlSimulators[gripper_index_];

	if ( controlled_simulator_ == nullptr )
	{
		ROS_ERROR("GripperInterfaceBase::GripperInterfaceBase - No controlled robot simulator available for gripper at index %d", gripper_index_);
		return;
	}

	// Filter out revolute joints, which will be normalized in later steps
	for ( size_t i=0; i<gripper_->links.size(); ++i )
	{
		if ( gripper_->links[i].type == RobotLink3D::Revolute )
			revolute_joint_indexes_.push_back(i);
	}

	std::vector<std::string> gripper_frames;
	std::vector<double> gripper_opened;
	std::vector<double> gripper_closed;
	std::string gripper_grasp_frame_name;
	nh->getParam("gripper_finger_frames", gripper_frames);
	nh->getParam("gripper_open_config", gripper_opened);
	gripper_opened_config_ = Config(gripper_opened);
	CalcUtils::normalizeConfiguration(gripper_opened_config_);
	nh->getParam("gripper_closed_config", gripper_closed);
	gripper_closed_config_ = Config(gripper_closed);
	CalcUtils::normalizeConfiguration(gripper_closed_config_);
	nh->param<std::string>("gripper_grasp_frame_name", gripper_grasp_frame_name, "");
	nh->param<double>("contact_force_threshold", contact_force_threshold_, 0.0);

	if ( contact_force_threshold_ < 0 )  // only values > 0 allowed
	{
		ROS_WARN("GripperInterfaceBase::GripperInterfaceBase - contact_force_threshold < 0, taking absolute value.");
		contact_force_threshold_ = std::abs(contact_force_threshold_);
	}

	if ( gripper_frames.size() == 0 || gripper_frames.size() != gripper_opened_config_.size() || gripper_closed_config_.size() != gripper_opened_config_.size() )
	{
		ROS_ERROR("GripperInterfaceBase::GripperInterfaceBase - gripper_open_config size %d does either not match gripper_frames size %d or gripper_closed_config size %d", (int)gripper_opened_config_.size(), (int)gripper_frames.size(), (int)gripper_closed_config_.size());
		return;
	}

	if ( gripper_grasp_frame_name.empty() )
	{
		ROS_ERROR("GripperInterfaceBase::GripperInterfaceBase - gripper_grasp_frame_name is empty.");
		return;
	}

	// Save gripper finger indexes for faster access later on
	gripper_finger_frame_indexes_.reserve(gripper_frames.size());
	for ( size_t i=0; i<gripper_->links.size(); ++i )
	{
		for ( size_t j=0; j<gripper_frames.size(); ++j )
		{
			if ( gripper_->LinkName(i).compare(gripper_frames[j]) == 0 )
				gripper_finger_frame_indexes_.push_back(i);
			else if ( gripper_->LinkName(i).compare(gripper_grasp_frame_name) == 0 )
				gripper_grasp_frame_index_ = i;
		}
	}

	if ( gripper_finger_frame_indexes_.size() != gripper_frames.size() )
	{
		ROS_ERROR("GripperInterfaceBase::GripperInterfaceBase - Could not find all gripper frames defined in yaml file.");
		return;
	}

	// Load grasp point position (defines the point the gripper aligns with the grasp pose in order to grasp) which is realtive to the grasp frame
	std::vector<double> grasp_point_position;
	nh->getParam("grasp_point_position", grasp_point_position);
	if ( grasp_point_position.size() != 3 )
	{
		ROS_ERROR("GripperInterfaceBase::GripperInterfaceBase - grasp_point_position has invalid size %d, expected 3.", (int)grasp_point_position.size());
		return;
	}
	grasp_point_position_.set(grasp_point_position[0], grasp_point_position[1], grasp_point_position[2]);

	// Load grasp alignment vector
	std::vector<double> grasp_direction_vector;
	nh->getParam("grasp_direction_vector", grasp_direction_vector);
	if ( grasp_direction_vector.size() != 3 )
	{
		ROS_ERROR("GripperInterfaceBase::GripperInterfaceBase - grasp_direction_vector has invalid size %d, expected 3.", (int)grasp_direction_vector.size());
		return;
	}
	grasp_direction_vector_.set(grasp_direction_vector[0], grasp_direction_vector[1], grasp_direction_vector[2]);

	// Load plane alignment vector
	std::vector<double> gripper_alignment_vector;
	nh->getParam("gripper_alignment_vector", gripper_alignment_vector);
	if ( gripper_alignment_vector.size() != 3 )
		ROS_WARN("GripperInterfaceBase::GripperInterfaceBase - gripper_alignment_vector has invalid size %d, expected 3.", (int)gripper_alignment_vector.size());
	else
		gripper_alignment_vector_.set(gripper_alignment_vector[0], gripper_alignment_vector[1], gripper_alignment_vector[2]);

	// Load other settings
	nh->param<double>("global_gripper_movement_timeout", global_gripper_movement_timeout_, 0.0);
	nh->param<double>("pre_grasp_offset", pre_grasp_offset_, 0.0);
	nh->param<double>("object_approaching_move_time", object_approaching_move_time_, 0.0);

	// Set up some extra margin for penetration avoidance
	const double extra_margin = 0.005;
	for( size_t i=0; i<gripper_->geometry.size(); ++i )
	{
		if ( gripper_->geometry[i] != nullptr )  // exclude links that do not have a geometry assigned
			gripper_->geometry[i]->margin += extra_margin;
	}

	initialized_ = true;
}

GripperInterfaceBase::~GripperInterfaceBase()
{
}

Config GripperInterfaceBase::getCurrentConfig()
{
	Config current_config;
	controlled_simulator_->GetSimulatedConfig(current_config);
	CalcUtils::normalizeConfiguration(current_config, revolute_joint_indexes_);
	return current_config;
}

Config GripperInterfaceBase::getGripperOpenedConfig()
{
	Config current_config;
	controlled_simulator_->GetSimulatedConfig(current_config);

	for ( size_t i=0; i<gripper_finger_frame_indexes_.size(); ++i )
	{
		const size_t &gripper_idx = gripper_finger_frame_indexes_[i];
		current_config[gripper_idx] = gripper_opened_config_[i];
	}

	CalcUtils::normalizeConfiguration(current_config, revolute_joint_indexes_);
	return current_config;
}

Config GripperInterfaceBase::getGripperClosedConfig()
{
	Config current_config;
	controlled_simulator_->GetSimulatedConfig(current_config);

	for ( size_t i=0; i<gripper_finger_frame_indexes_.size(); ++i )
	{
		const size_t &gripper_idx = gripper_finger_frame_indexes_[i];
		current_config[gripper_idx] = gripper_closed_config_[i];
	}

	CalcUtils::normalizeConfiguration(current_config, revolute_joint_indexes_);
	return current_config;
}

bool GripperInterfaceBase::isGripperOpen()
{
	Config opened_config = getGripperOpenedConfig();
	return hasReachedTarget(opened_config);
}

bool GripperInterfaceBase::isGripperClosed()
{
	for ( size_t i=0; i<gripper_finger_frame_indexes_.size(); ++i )  // Check each link whether it surpasses the force threshold
	{
		double link_force = simulation_.ContactForce(world_.RobotLinkID(gripper_index_, gripper_finger_frame_indexes_[i])).norm();

		if ( link_force > contact_force_threshold_ )  // force threshold has been reached -> consider state as closed
			return true;
	}

	// Check whether gripper has reached its maximum closed state
	Config closed_config = getGripperClosedConfig();
	return hasReachedTarget(closed_config);
}

bool GripperInterfaceBase::setMoveToConfig(Config &target_config)
{
	Config current_config;
	controlled_simulator_->GetSimulatedConfig(current_config);

	if ( target_config.size() != current_config.size() )
		return false;

	CalcUtils::normalizeConfiguration(target_config, revolute_joint_indexes_);
	controller_->SendCommand("set_q", LexicalCast(target_config));
	return true;
}

bool GripperInterfaceBase::setMoveToCartesian(const RigidTransform &target_pose, Config &target_config, const double &move_time, SimGUIBackend* gui_backend)
{
	Config current_config;
	controlled_simulator_->GetSimulatedConfig(current_config);

	if ( !computeConfigFromCartesian(target_pose, target_config) )
		return false;

	if ( move_time < 0.1 )  // drive at full speed
	{
		controller_->SendCommand("set_q", LexicalCast(target_config));
		return true;
	}
	else if ( gui_backend != nullptr )  // drive at defined speed
	{
		std::vector<double> times(2, 0);
		CalcUtils::normalizeConfiguration(current_config, revolute_joint_indexes_);
		std::vector<Config> milestones(2, current_config);
		times[1] = move_time;
		milestones[1] = target_config;
		gui_backend->SendLinearPath(times, milestones);
		return true;
	}

	return false;
}

bool GripperInterfaceBase::computeConfigFromCartesian(const RigidTransform &target_pose, Config &target_config)
{
	Config current_config;
	controlled_simulator_->GetSimulatedConfig(current_config);

	IKGoal goal;
	goal.link = (int)gripper_grasp_frame_index_;
	goal.localPosition = grasp_point_position_;  // position of end effector in local frame.
	goal.SetFixedPosition(target_pose.t);  // end position in world coordinates
	goal.SetFixedRotation(target_pose.R);

	double tolerance = 1e-5;
	int iters = 100;
	const int verbose = 0;
	vector<IKGoal> problem;
	problem.push_back(goal);

	const bool result = SolveIK(*gripper_, problem, tolerance, iters, verbose);  // SolveIK stores result in robot->q

	if ( !result )
	{
		ROS_WARN("SimulatorEnvironment::computeConfigFromCartesian - Failed solving the inverse kinematics problem.");
		return false;
	}

	target_config.copy(gripper_->q);  // save config of desired pose
	CalcUtils::normalizeConfiguration(target_config, revolute_joint_indexes_);
	gripper_->UpdateConfig(current_config);  // reset robot's config
	return true;
}

void GripperInterfaceBase::setOpenGripper()
{
	Config opened_config = getGripperOpenedConfig();
	setMoveToConfig(opened_config);
}

void GripperInterfaceBase::addOpenGripperToConfig(Config &append_to)
{
	for ( size_t i=0; i<gripper_finger_frame_indexes_.size(); ++i )
	{
		const size_t &gripper_idx = gripper_finger_frame_indexes_[i];
		append_to[gripper_idx] = gripper_opened_config_[i];
	}

	CalcUtils::normalizeConfiguration(append_to, revolute_joint_indexes_);
}

bool GripperInterfaceBase::hasReachedTarget(const Config &target_config, const bool &ignore_fingers)
{
	Config current_config = getCurrentConfig();
	Config accuracy_requirements = getPerJointAccuracyParameters();
	Config target_config_modified = target_config;

	if ( ignore_fingers )  // overwrite finger links so they are alike
	{
		for ( size_t idx : gripper_finger_frame_indexes_ )
		{
			if ( idx < target_config_modified.size() && idx < current_config.size() )
				target_config_modified[idx] = current_config[idx];
		}
	}

	return CalcUtils::equalConfigs(current_config, target_config_modified, accuracy_requirements, revolute_joint_indexes_);
}

bool GripperInterfaceBase::isInitialized()
{
	return initialized_;
}

void GripperInterfaceBase::getWorldToGraspFrame(Math3D::RigidTransform &world_to_grasp_frame)
{
	world_to_grasp_frame.setIdentity();
	world_to_grasp_frame = gripper_->links[gripper_grasp_frame_index_].T_World;
}

void GripperInterfaceBase::stopMovement()  // Move to current config which stops the movement
{
	Config current_config = getCurrentConfig();
	controlled_simulator_[gripper_index_].command.ResetPIDIntegrals();  // Reset integrator values so they do not overflow over time
	setMoveToConfig(current_config);
}

// Returns a vector in the grasp frame that shall be used to align the gripper to the inverted normal vector of each grasp pose (i.e. the vector that points towards the surface)
// The vector is so to say the direction (in relation to the grasp frame) in which the gripper can perform a grasp
// The vector will be normalized.
Math3D::Vector3 GripperInterfaceBase::getGraspDirectionVector()
{
	Math3D::Vector3 grasp_direction_normalized;
	grasp_direction_vector_.getNormalized(grasp_direction_normalized);
	return grasp_direction_normalized;
}

// Returns the vector in intermediate frame coordinates which is used to achieve a second alignment.
// The intermediate frame results from the alignment of the grasp alignment vector and the inverted normal.
// This vector will then be aligned such that it is parallel to a given plane (e.g. ground) by only rotating around the inverted normal.
// This second alignment is optional and can be turned off by returning a 0-vector.
// The vector will be normalized as well.
Math3D::Vector3 GripperInterfaceBase::getGripperAlignmentVector()
{
	Math3D::Vector3 gripper_alignment_normalized;
	gripper_alignment_vector_.getNormalized(gripper_alignment_normalized);
	return gripper_alignment_normalized;
}

// Shifts the pre grasp position of the gripper along the surface normal away from the actual grasp position [in m]
// Only positive values are allowed (absolute value is taken internally) which shift in direction of the surface normal
double GripperInterfaceBase::getPreGraspOffset()
{
	return pre_grasp_offset_;
}

// Time the gripper will need to drive from pre grasp pose to the grasp pose
double GripperInterfaceBase::getObjectApproachingMoveTime()
{
	return object_approaching_move_time_;
}

double GripperInterfaceBase::getGripperMovementTimeout(int simulation_state)
{
	return global_gripper_movement_timeout_;
}

int GripperInterfaceBase::getGripperID()
{
	return world_.RobotID(gripper_index_);
}

bool GripperInterfaceBase::hasContactWithRigidObject()
{
	for( size_t i=0; i<world_.rigidObjects.size(); i++ )  // whether in contact with any rigid objects
	{
		for( size_t j=0; j<gripper_->links.size(); j++ )
		{
			if ( simulation_.InContact(world_.RigidObjectID(i), world_.RobotLinkID(gripper_index_, j)) )
				return true;
		}
	}

	return false;
}

bool GripperInterfaceBase::maxContactForceExceeded()
{
	for ( size_t i=0; i<gripper_->links.size(); ++i )
	{
		const double current_force = simulation_.ContactForce(world_.RobotLinkID(gripper_index_, i)).norm();
		if ( current_force > MAX_ALLOWED_FORCE )
			return true;
	}

	return false;
}

std::vector<size_t> GripperInterfaceBase::getRevoluteJointIndexes()
{
	return revolute_joint_indexes_;
}
