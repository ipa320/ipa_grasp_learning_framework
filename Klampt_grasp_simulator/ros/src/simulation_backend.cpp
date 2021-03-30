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

// Class that contains all settings and functions concerning the simulation
// Loads necessary properties from the ros node handle and holds the simulation logic


#include <Klampt_grasp_simulator/simulation_backend.h>
#include <Klampt_grasp_simulator/file_utilities.h>
#include <ipa_grasppose_srvs/GetGraspPoses.h>
#include <ipa_grasppose_srvs/SaveGraspData.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <utility>


// ToDo: Create new settings from constants (accuracy parameters in advanceSimulation, delta time) or put them in gripper_interface
// ToDo: Debug when more objects than 1 are spawned -> contact feedback deletion [Done]
// ToDo: Add a timeout in ST_OBJECT_WAIT when an object fails to settle, or sth. else to prevent an infinite waiting
// ToDo: Remove deprecated functions from gipper interface
// ToDo: Remove create_gui flag, it does not work well
// ToDo: Move dt_ to simulation yaml file
// ToDo: Save perspective_augmentation_counter_ to file as well, needed for recovering on crash [Done]
// ToDo: Think more about isGraspPoseWithinPointCloud and how to remove false positives/true negatives. [Done]
// ToDo: Check interfaces and other nodes for hardcoded constants


// DEBUGGING
#include <KrisLibrary/GLdraw/GLScreenshot.h>
#include <ipa_grasppose_srvs/PredictGraspQualities.h>
ros::Publisher point_cloud_2_pub;
ros::Publisher depth_image_pub;
// End debugging

// Constants
#define STUCK_DETECTION_SAMPLING_FACTOR 5	// multiplied with dt_ to define the check time of stuck behaviour
#define STUCK_COUNT_THRESHOLD 3				// if gripper has been stuck more than or equal to this value -> trigger stuck recover
#define OBJECT_DEVIATION_STRICT_POS 0.01	// allowed object deviation regarding position (in meters) in ST_GRIPPER_MOVE_TO_GRASP
#define OBJECT_DEVIATION_STRICT_ROT 0.05	// allowed object deviation regarding rotation (in radian) in ST_GRIPPER_MOVE_TO_GRASP
#define OBJECT_DEVIATION_SOFT_POS 0.03		// allowed object deviation regarding position (in meters) in ST_GRIPPER_CLOSING, ST_GRIPPER_LIFT_OBJECT, ST_ASSESS_GRASP
#define OBJECT_DEVIATION_SOFT_ROT 0.15		// allowed object deviation regarding rotation (in radian) in ST_GRIPPER_CLOSING, ST_ASSESS_GRASP
#define GRIPPER_DEVIATION_POS 0.005			// allowed gripper deviation regarding position (in meters) in ST_GRIPPER_CLOSING
#define GRIPPER_DEVIATION_ROT 0.03			// allowed object deviation regarding rotation (in radian) in ST_GRIPPER_CLOSING
#define OBJECT_VELOCITY_ANGULAR 0.01		// allowed angular velocity (in rad/s) of object in ST_ASSESS_GRASP
#define OBJECT_VELOCITY_LINEAR 0.005		// allowed linear velocity (in m/s) of object in ST_ASSESS_GRASP


SimulationBackend::SimulationBackend() :
					gui_backend_(nullptr)
{
}

SimulationBackend::SimulationBackend(ros::NodeHandle* nh, RobotWorld* world) :
					node_handle_(nh), world_(world), gui_backend_(world), collision_detection_(nullptr), gripper_interface_(nullptr), camera_index_(0), camera_(nullptr),
					current_rigid_object_defaults_(nullptr), simulation_state_(SimulationState(0)), camera_flight_counter_(0), current_camera_flight_(nullptr),
					current_delta_poses_set_(nullptr), perspective_augmentation_counter_(0), saved_perspective_augmentations_(0),
					perspective_augmentation_prepared_(false), recover_to_perspective_augmentation_state_(false), gripper_moves_(false),
					gripper_poses_counter_(0), skip_evasion_once_(false), simulation_has_finished_(false), dt_(0.1), world_path_(""), rigid_objects_path_(""),
					rigid_object_database_counter_(0), current_rigid_object_(nullptr), current_grasp_pose_(nullptr), error_flag_(false), last_stuck_check_time_(0),
					last_object_movement_time_(0), grasp_assessment_time_(0), movement_timeout_time_(0), stuck_counter_(0), plane_normal_filter_world_(0,0,0),
					ignore_last_grasp_(false)
{
	simulation_ = &(gui_backend_.sim);
	last_stamped_gripper_config_.time_stamp = 0;
	original_object_aabb_center_.setZero();

	// Add node name to simulation progress storage file, so this node can be started several times using different names
	std::vector<std::string> result = FileUtils::string_split(ros::this_node::getName(), "/");

	if ( !result.empty() )
		simulation_storage_file_ = result.back()+"_simulation_progress.txt";
	else
		simulation_storage_file_ = "simulation_progress.txt";
}

SimulationBackend::~SimulationBackend()
{
	if ( gripper_interface_ != nullptr )
		delete gripper_interface_;

	if ( collision_detection_ != nullptr )
		delete collision_detection_;
}

bool SimulationBackend::initBackend()
{
	node_handle_->param<std::string>("storage_folder_path", storage_folder_path_, "");
	if ( !FileUtils::checkCreateFolder(storage_folder_path_) )
	{
		ROS_ERROR("SimulatorEnvironment::setupSimulator - Could not create storage folder.");
		return false;
	}
	simulation_storage_file_path_ = storage_folder_path_+"/"+simulation_storage_file_;

	node_handle_->param<std::string>("world", world_path_, "");
	if ( world_path_.empty() )
	{
		ROS_ERROR("SimulatorEnvironment::setupSimulator - world path is empty!");
		return false;
	}

	// Load xml world file
	if ( !gui_backend_.LoadAndInitSim(world_path_.c_str()) )
	{
		ROS_ERROR("SimulatorEnvironment::setupSimulator - Error loading world file: %s.", world_path_.c_str());
		return false;
	}

	if ( simulation_->robotControllers.size() == 0 )
	{
		ROS_ERROR("SimulatorEnvironment::setupSimulator - No robot controllers have been loaded.");
		return false;
	}

	std::string gripper_interface_name;
	node_handle_->param<std::string>("gripper_interface_name", gripper_interface_name, "");
	gripper_interface_ = GripperInterfaceBase::createGripperInterfaceByName(gripper_interface_name, node_handle_, *world_, *simulation_);
	if ( gripper_interface_ == nullptr || !gripper_interface_->isInitialized() )
	{
		ROS_ERROR("SimulatorEnvironment::setupSimulator - Failed to initialize the gripper interface.");
		return false;
	}

	// Save startup configuration and position (in world frame) for later usage
	gripper_start_configuration_ = gripper_interface_->getCurrentConfig();
	Math3D::RigidTransform grasp_frame_world;
	gripper_interface_->getWorldToGraspFrame(grasp_frame_world);
	gripper_start_pose_ = grasp_frame_world;

	// Store pointer to camera robot
	node_handle_->param<int>("camera_index", camera_index_, 0);

	if ( camera_index_ >=0 && camera_index_ < world_->robots.size() )
		camera_ = world_->robots[camera_index_];  // store camera for faster access

	if ( camera_ == nullptr )
	{
		ROS_ERROR("SimulatorEnvironment::setupSimulator - camera_index %d is out of bounds.", camera_index_);
		return false;
	}

	// Retrieve the camera sensor from the "camera robot"
	std::vector<CameraSensor*> cameras;
	simulation_->controlSimulators[camera_index_].sensors.GetTypedSensors(cameras);

	if ( cameras.empty() )
	{
		ROS_ERROR("SimulatorEnvironment::setupSimulator - No camera sensor has been found, please add one to the camera robot in the world xml file.");
		return false;
	}

	if ( cameras.size() > 1 )
		ROS_WARN("SimulatorEnvironment::setupSimulator - Multiple camera sensors have been found, working with first one.");

	camera_sensor_ = cameras[0];

	if ( !camera_sensor_->rgb || !camera_sensor_->depth )
	{
		ROS_ERROR("SimulatorEnvironment::setupSimulator - Camera sensor must provide rgb and depth information, please enable both.");
		return false;
	}

	toggleCameraState(false);  // deactivate camera to speed up simulation time

	// Generate all loaded active camera flights
	std::vector<std::string> active_camera_flights;
	node_handle_->getParam("active_camera_flights", active_camera_flights);
	if ( active_camera_flights.empty() )
	{
		ROS_ERROR("SimulatorEnvironment::setupSimulator - active_camera_flights parameter is empty!");
		return false;
	}

	// Generate all delta poses of the perspective augmentation of the camera.
	// Each perspective augmentation consists of a delta pose (delta position vector in camera frame and absolute rotation).
	// The delta poses are later during simulation added on top of each camera flight pose to diversify the depth images.
	// Each perspective augmentation keeps the same sampled grasp poses and their evaluations as the corresponding camera flight pose.
	// Therefore, this method saves a lot of simulation time since no extra grasp evaluations are needed but grants diversification.
	bool use_perspective_augmentation = false;
	node_handle_->param<bool>("use_perspective_augmentation", use_perspective_augmentation, false);
	std::vector<double> perspective_augmentation_angles;  // used later on to generate the delta poses if use_perspective_augmentation == true
	Math3D::RigidTransform camera_to_world;  // also used later if use_perspective_augmentation == true
	const double reference_phi = 0.0;  // reference phi: Together with reference_theta it defines a reference position on the sphere which is used to pre-build the delta poses
	const double reference_theta = Math::Pi_2;
	Math3D::Matrix3 sensor_to_camera_at_reference_angles;  // used later on if use_perspective_augmentation == true to generate the delta poses for the reference position

	if ( use_perspective_augmentation )
	{
		// The sensor is coupled to the camera link: world -> T_camera -> T_Sensor
		// T_camera is a fix transformation that does not change over time and converts the world frame into the ros standard frame used for cameras (camera link).
		double pan_angle = 0.0, tilt_angle = 0.0;
		CalcUtils::getPanTiltFromSphericalCoordinates(reference_phi, reference_theta, pan_angle, tilt_angle);
		const Math3D::EulerAngleRotation euler_angles(pan_angle, tilt_angle, 0.0);
		euler_angles.getMatrixYXZ(sensor_to_camera_at_reference_angles);
		sensor_to_camera_at_reference_angles.inplaceInverse();
		camera_->links[camera_sensor_->link].T_World.getInverse(camera_to_world);

		node_handle_->getParam("perspective_augmentation_angles", perspective_augmentation_angles);
		if ( perspective_augmentation_angles.size() != 4 )
		{
			ROS_ERROR("SimulatorEnvironment::setupSimulator - perspective_augmentation_angles has wrong size %d, expected 4.", (int)perspective_augmentation_angles.size());
			return false;
		}

		// Check for valid parameters
		double &d_theta = perspective_augmentation_angles[0];  // delta theta angle
		double &d_theta_sub = perspective_augmentation_angles[1];
		double &d_phi_sub = perspective_augmentation_angles[2];
		double &d_gamma_sub = perspective_augmentation_angles[3];

		if ( !CalcUtils::forceRangeMinMax(d_theta, 0.0, Math::Pi_4) )
			ROS_WARN("SimulatorEnvironment::setupSimulator - perspective_augmentation_angles' delta theta exceeds [0,pi/4], set to comply with boundary.");

		// Avoid errors in subdivisions parameters that can lead to infinite loops
		if ( d_theta_sub < 1.0 )
		{
			d_theta_sub = 1.0;
			ROS_WARN("SimulatorEnvironment::setupSimulator - perspective_augmentation_angles' delta theta subdivisions are invalid, set to 1.");
		}

		if ( d_phi_sub < 1.0 )
		{
			d_phi_sub = 1.0;
			ROS_WARN("SimulatorEnvironment::setupSimulator - perspective_augmentation_angles' delta phi subdivisions are invalid, set to 1.");
		}

		if ( d_gamma_sub < 1.0 )
		{
			d_gamma_sub = 1.0;
			ROS_WARN("SimulatorEnvironment::setupSimulator - perspective_augmentation_angles' delta gamma subdivisions are invalid, set to 1.");
		}
	}

	// Compute cartesian positions and camera rotation for each camera flight
	for ( std::string camera_flight_name : active_camera_flights )
	{
		std::vector<double> camera_flight;
		node_handle_->getParam(camera_flight_name, camera_flight);
		if ( camera_flight.size() != 12 )
		{
			ROS_ERROR("SimulatorEnvironment::setupSimulator - %s has wrong size %d, expected 12.", camera_flight_name.c_str(), (int)camera_flight.size());
			return false;
		}

		double &radius_start = camera_flight[0];
		double &radius_step = camera_flight[1];
		double &radius_end = camera_flight[2];
		double &phi_start = camera_flight[3];
		double &phi_step = camera_flight[4];
		double &phi_end = camera_flight[5];
		double &theta_start = camera_flight[6];
		double &theta_step = camera_flight[7];
		double &theta_end = camera_flight[8];
		double &gamma_start = camera_flight[9];
		double &gamma_step = camera_flight[10];
		double &gamma_end = camera_flight[11];

		// Check for valid ranges
		if ( !CalcUtils::forceRangeMinMax(radius_start, 0.01, 1000) )
			ROS_WARN("SimulatorEnvironment::setupSimulator - camera_flight's radius start exceeds [0.01,1000], set to comply with boundary.");

		if ( !CalcUtils::forceRangeMinMax(radius_end, 0.01, 1000) )
			ROS_WARN("SimulatorEnvironment::setupSimulator - camera_flight's radius end exceeds [0.01,1000], set to comply with boundary.");

		if ( !CalcUtils::forceRangeAbs(phi_start, Math::Pi) )
			ROS_WARN("SimulatorEnvironment::setupSimulator - camera_flight's phi start exceeds [-pi,pi], set to comply with boundary.");

		if ( !CalcUtils::forceRangeAbs(phi_end, Math::Pi) )
			ROS_WARN("SimulatorEnvironment::setupSimulator - camera_flight's phi end exceeds [-pi,pi], set to comply with boundary.");

		if ( !CalcUtils::forceRangeMinMax(theta_start, 0, Math::Pi) )
			ROS_WARN("SimulatorEnvironment::setupSimulator - camera_flight's theta start exceeds [0,pi], set to comply with boundary.");

		if ( !CalcUtils::forceRangeMinMax(theta_end, 0, Math::Pi) )
			ROS_WARN("SimulatorEnvironment::setupSimulator - camera_flight's theta end exceeds [0,pi], set to comply with boundary.");

		if ( !CalcUtils::forceRangeMinMax(gamma_start, 0, Math::TwoPi) )
			ROS_WARN("SimulatorEnvironment::setupSimulator - camera_flight's gamma start exceeds [0,2*pi], set to comply with boundary.");

		if ( !CalcUtils::forceRangeMinMax(gamma_end, 0, Math::TwoPi) )
			ROS_WARN("SimulatorEnvironment::setupSimulator - camera_flight's gamma end exceeds [0,2*pi], set to comply with boundary.");

		// Avoid errors in range parameters that can lead to infinite loops
		if ( !CalcUtils::checkAndFixStep(radius_start, radius_end, radius_step) )
			ROS_WARN("SimulatorEnvironment::setupSimulator - camera_flight's radius range is badly configured, negating step value.");

		if ( !CalcUtils::checkAndFixStep(phi_start, phi_end, phi_step) )
			ROS_WARN("SimulatorEnvironment::setupSimulator - camera_flight's phi range is badly configured, negating step value.");

		if ( !CalcUtils::checkAndFixStep(theta_start, theta_end, theta_step) )
			ROS_WARN("SimulatorEnvironment::setupSimulator - camera_flight's theta range is badly configured, negating step value.");

		if ( !CalcUtils::checkAndFixStep(gamma_start, gamma_end, gamma_step) )
			ROS_WARN("SimulatorEnvironment::setupSimulator - camera_flight's gamma range is badly configured, negating step value.");

		// Sample in spherical coordinates and transform them back to cartesian coordinates
		std::vector<Math3D::RigidTransform> camera_flight_poses;
		for ( double radius=radius_start; radius-radius_end<0.0001; radius+=radius_step )  // take into account inaccuracies due to floating point operations -> use bounds such as 0.0001
		{
			for ( double theta=theta_start; (theta_step >= 0.0) ? theta-theta_end<0.0001 : theta-theta_end>-0.0001; theta+=theta_step )  // make this work for positive and negative step values -> conditional operator
			{
				for ( double phi=phi_start; (phi_step >= 0.0) ? phi-phi_end<0.0001 : phi-phi_end>-0.0001; phi+=phi_step )
				{
					Math3D::RigidTransform pose;
					CalcUtils::convertSphericalToCartesian(radius, phi, theta, pose.t);

					// These calculation demand that the camera frame complies the ros standard.
					// This only works well with phi, theta within [-pi,pi]
					double pan_angle = 0.0, tilt_angle = 0.0;
					CalcUtils::getPanTiltFromSphericalCoordinates(phi, theta, pan_angle, tilt_angle);

					for ( double gamma=gamma_start; (gamma_step >= 0.0) ? gamma-gamma_end<0.0001 : gamma-gamma_end>-0.0001; gamma+=gamma_step )  // roll around camera's optical line
					{
						Math3D::EulerAngleRotation rotation(pan_angle, tilt_angle, gamma);
						rotation.getMatrixYXZ(pose.R);
						camera_flight_poses.push_back(pose);  // local positions, but global rotation
					}

					if ( std::abs(theta) <= 0.0001 || std::abs(theta-Math::Pi) <= 0.005 )  // theta == 0 or pi -> phi and gamma do the same -> one phi with all gammas is enough
						break;
				}
			}

			// Add perspective augmentation in here
			if ( use_perspective_augmentation )
				checkAddToDeltaPosesSet(radius, perspective_augmentation_angles, reference_phi, reference_theta, sensor_to_camera_at_reference_angles, camera_to_world.R);
		}

		if ( camera_flight_poses.empty() )
		{
			ROS_ERROR("SimulatorEnvironment::setupSimulator - Generated camera flight of %s is empty, skipping.", camera_flight_name.c_str());
			return false;
		}
		else
			active_camera_flights_.push_back(camera_flight_poses);
	}

	// Set up pointcloud conversion, so this has to only be calculated once
	int &height = camera_sensor_->yres;
	int &width = camera_sensor_->xres;
	pcl_conversion_.x_shift = -width*0.5;  // shift origin to center of image
	pcl_conversion_.y_shift = -height*0.5;
	pcl_conversion_.x_scale = std::tan(camera_sensor_->xfov*0.5) / (width*0.5);
	//pcl_conversion_.y_scale = -1.0/(std::tan(camera_sensor_->yfov*0.5)*height*0.5)
	pcl_conversion_.y_scale = pcl_conversion_.x_scale;  // square pixels are assumed
	pcl_conversion_.allowed_AABB_deviation = 0.01;  // up to 1cm deviation to AABB is allowed

	// Set up CameraInfo topic
	CalcUtils::convertToCameraInfo(*camera_sensor_, camera_info_);

	// Set up color and distance filter, so that the object can be segmented out of the point cloud properly
	node_handle_->param<bool>("use_color_filter", pcl_conversion_.use_color_filter, false);
	if ( pcl_conversion_.use_color_filter )
	{
		std::vector<double> allowed_deviation_hsv;
		node_handle_->getParam("allowed_deviation_hsv", allowed_deviation_hsv);
		if ( allowed_deviation_hsv.size() != 3 )
		{
			ROS_ERROR("SimulatorEnvironment::setupSimulator - Invalid allowed_deviation_hsv size %d, expected 3.", (int)allowed_deviation_hsv.size());
			return false;
		}

		if ( !CalcUtils::forceRangeMinMax(allowed_deviation_hsv[0], 0, 360) )
			ROS_WARN("SimulatorEnvironment::setupSimulator - allowed_deviation_hsv's h value exceeds [0,360], set to comply with boundary.");

		if ( !CalcUtils::forceRangeMinMax(allowed_deviation_hsv[1], 0, 1) )
			ROS_WARN("SimulatorEnvironment::setupSimulator - allowed_deviation_hsv's s value exceeds [0,1], set to comply with boundary.");

		if ( !CalcUtils::forceRangeMinMax(allowed_deviation_hsv[2], 0, 1) )
			ROS_WARN("SimulatorEnvironment::setupSimulator - allowed_deviation_hsv's v value exceeds [0,1], set to comply with boundary.");

		pcl_conversion_.allowed_deviation_hsv[0] = allowed_deviation_hsv[0];
		pcl_conversion_.allowed_deviation_hsv[1] = allowed_deviation_hsv[1];
		pcl_conversion_.allowed_deviation_hsv[2] = allowed_deviation_hsv[2];

		std::vector<int> rgb_color;
		node_handle_->getParam("rgb_filter", rgb_color);
		if ( rgb_color.size() != 3 )
		{
			ROS_ERROR("SimulatorEnvironment::setupSimulator - Invalid rgb_filter size %d, expected 3.", (int)rgb_color.size());
			return false;
		}

		if ( !CalcUtils::forceRangeMinMax(rgb_color[0], 0, 255) )
			ROS_WARN("SimulatorEnvironment::setupSimulator - rgb_filter's r value exceeds [0,255], set to comply with boundary.");

		if ( !CalcUtils::forceRangeMinMax(rgb_color[1], 0, 255) )
			ROS_WARN("SimulatorEnvironment::setupSimulator - rgb_filter's g value exceeds [0,255], set to comply with boundary.");

		if ( !CalcUtils::forceRangeMinMax(rgb_color[2], 0, 255) )
			ROS_WARN("SimulatorEnvironment::setupSimulator - rgb_filter's b value exceeds [0,255], set to comply with boundary.");

		pcl_conversion_.rgb_color.red = rgb_color[0];
		pcl_conversion_.rgb_color.green = rgb_color[1];
		pcl_conversion_.rgb_color.blue = rgb_color[2];
		CalcUtils::convertRGBToHSV(pcl_conversion_.rgb_color, pcl_conversion_.hsv_converted);
	}

	node_handle_->param<bool>("use_boundingbox_filter", pcl_conversion_.use_boundingbox_filter, false);

	// Load database from parameter server and process it
	if ( !setupRigidObjectDatabase() )
		return false;

	// Retrieve object index of previous run that has not completed successfully
	if ( FileUtils::pathExists(simulation_storage_file_path_) )
	{
		ROS_WARN("Loading previous session from file...");
		if ( !loadSimulationDataFromDrive() )
			return false;
	}

	node_handle_->param<std::string>("objects", rigid_objects_path_, "");
	if ( rigid_objects_path_.empty() )
	{
		ROS_ERROR("SimulatorEnvironment::setupSimulator - objects path is empty!");
		return false;
	}

	// Load floor normal in world coordinates
	std::vector<double> plane_normal_filter_world;
	node_handle_->getParam("plane_normal_filter_world", plane_normal_filter_world);
	if ( plane_normal_filter_world.size() != 3 )
		ROS_WARN("SimulatorEnvironment::setupSimulator - plane_normal_filter_world has invalid size %d, expected 3.", (int)plane_normal_filter_world.size());
	else
		plane_normal_filter_world_.set(plane_normal_filter_world[0], plane_normal_filter_world[1], plane_normal_filter_world[2]);

	// Set up collision detection object
	collision_detection_ = new WorldPlannerSettings();
	collision_detection_->InitializeDefault(*world_);

	// Grasppose sampler service settings
	node_handle_->param<double>("normal_search_radius", normal_search_radius_, 0.01);
	node_handle_->param<float>("downsampling_voxel_leaf_size", downsampling_voxel_leaf_size_, 0.0);

	// Simulation related options
	node_handle_->param<bool>("draw_gripper_graspframe", draw_gripper_graspframe_, false);
	node_handle_->param<bool>("draw_all_graspposes", draw_all_graspposes_, false);
	node_handle_->param<bool>("draw_current_grasppose", draw_current_grasppose_, false);
	node_handle_->param<bool>("draw_bounding_box", draw_bounding_box_, false);
	node_handle_->param<bool>("draw_camera", draw_camera_, false);
	node_handle_->param<bool>("draw_camera_frustum", draw_camera_frustum_, false);

	// DEGUGGING
	point_cloud_2_pub = node_handle_->advertise<sensor_msgs::PointCloud2>("Depth_points", 1, true);
	depth_image_pub = node_handle_->advertise<sensor_msgs::Image>("Depth_image", 1, true);
	prepare_errors_ = 0;
	all_grasps_ = 0;
	valid_grasps_ = 0;
	successful_grasps_ = 0;
	start_evasions_ = 0;
	end_evasions_ = 0;
	evasion_errors_ = 0;
	move_start_errors_ = 0;
	total_grasp_poses_ = 0;
	failed_save_calls_ = 0;

	ros::Duration(5.0).sleep();  // wait for nn node to be ready
	//camera_flight_counter_ = 1;
	//gripper_poses_counter_ = 343;
	// End debugging

	// Subscribers
	sample_graspposes_client_ = node_handle_->serviceClient<ipa_grasppose_srvs::GetGraspPoses>("/ipa_grasppose_sampler/ipa_grasppose_sampler/sample_graspposes_srv");
	save_graspposedata_client_ = node_handle_->serviceClient<ipa_grasppose_srvs::SaveGraspData>("/ipa_graspdata_manager/ipa_graspdata_manager/save_graspdata_srv");

	// Publishers
	camera_info_publisher_ = node_handle_->advertise<sensor_msgs::CameraInfo>("CameraInfo", 1, true);  // allow latching, so we do not have to iteratively send the camera_info topic acress the network
	camera_info_publisher_.publish(camera_info_);  // publish camera info topic of simulated camera once, latch flag will take care it will be distributed to new subscribers

	initialized_ = true;
	return true;
}

void SimulationBackend::checkAddToDeltaPosesSet(const double &radius, const std::vector<double> &delta_angles, const double &reference_phi, const double &reference_theta,
												const Math3D::Matrix3 &sensor_to_camera_at_reference_angles, const Math3D::Matrix3 &camera_to_world)
{
	// Check previous delta pose sets whether the current radius already exists
	for ( DeltaPosesSet set : perspective_augmentation_ )
	{
		if ( std::abs(set.radius - radius) < 0.00001 )  // radius has already been covered, exit
			return;
	}

	// Radius has not been found, add it to the list
	DeltaPosesSet delta_poses_set;
	delta_poses_set.radius = radius;

	// Reference angle information
	const double &d_theta = delta_angles[0];  // delta theta angle
	const double &d_theta_sub = delta_angles[1];
	const double &d_phi_sub = delta_angles[2];
	const double &d_gamma_sub = delta_angles[3];

	// Create step angles
	const double d_theta_step = (d_theta > 0.0) ? d_theta / std::abs((int)d_theta_sub) : 1.0;  // avoid step of 0 when delta theta is 0 -> infinite loop
	const double d_phi_step = Math::TwoPi / std::abs((int)d_phi_sub);  // always sample around full circle
	const double d_gamma_step = Math::TwoPi / std::abs((int)d_gamma_sub);

	// Generate local poses and transform them into the reference frame
	// The local poses lie around [0,0] in spherical coordinates
	Math3D::Matrix3 world_to_reference_rot;
	Math3D::EulerAngleRotation euler_angles(reference_phi, reference_theta, 0.0);
	euler_angles.getMatrixZYX(world_to_reference_rot);

	Math3D::Vector3 reference_position_world;
	CalcUtils::convertSphericalToCartesian(radius, reference_phi, reference_theta, reference_position_world);
	for ( double theta=0.0; theta-d_theta<0.0001; theta+=d_theta_step )
	{
		for ( double phi=0.0; phi-Math::TwoPi<-0.0001; phi+=d_phi_step )  // exclude 2*pi here and values very close to it (due to floating point operations this is rarely exact -> use -0.0001)
		{
			// Get pose in local coordinates
			Math3D::Vector3 delta_position_local;
			CalcUtils::convertSphericalToCartesian(radius, phi, theta, delta_position_local);

			Math3D::Vector3 delta_position_world = world_to_reference_rot * delta_position_local;  // rotate local position so that it lies around the reference position (in "world frame")
			double phi_world = 0.0, theta_world = 0.0;
			CalcUtils::convertCartesianToSpherical(delta_position_world, radius, phi_world, theta_world);

			// Transform delta position into the given sensor frame
			Math3D::RigidTransform pose;
			pose.t = sensor_to_camera_at_reference_angles * camera_to_world * (delta_position_world - reference_position_world);

			// Compute the absolute camera rotation
			// These angles are relative to the camera link
			double pan_angle = 0.0, tilt_angle = 0.0;
			CalcUtils::getPanTiltFromSphericalCoordinates(phi_world, theta_world, pan_angle, tilt_angle);

			for ( double gamma=0.0; gamma-Math::TwoPi<-0.0001; gamma+=d_gamma_step )
			{
				if ( std::abs(phi) <= 0.0001 && std::abs(theta) <= 0.0001 && std::abs(gamma) <= 0.0001 )  // exclude cameras original pose
					continue;

				Math3D::EulerAngleRotation rotation(pan_angle, tilt_angle, gamma);
				rotation.getMatrixYXZ(pose.R);

				// Transform rotation into given sensor frame to get the delta rotation and store pose
				pose.R = sensor_to_camera_at_reference_angles * pose.R;
				delta_poses_set.delta_poses.push_back(pose);
			}

			if ( std::abs(theta) <= 0.0001 || std::abs(theta-Math::Pi) <= 0.005 )  // theta == 0 or pi -> phi and gamma do the same -> one phi with all gammas is enough
				break;
		}
	}

	perspective_augmentation_.push_back(delta_poses_set);
}

bool SimulationBackend::setupRigidObjectDatabase()
{
	std::vector<std::string> database;
	node_handle_->getParam("graspable_objects", database);

	if ( database.empty() || database.size() % 9 != 0 )
	{
		ROS_ERROR("SimulatorEnvironment::setupRigidObjectDatabase - graspable_objects vector has wrong size %d, it must be a multiple of 9 according to [file_name, name_tag, camera_flight_index, x, y, z, roll, pitch, yaw].", (int)database.size());
		return false;
	}

	rigid_object_database_.reserve(database.size());
	for ( size_t i=0; i<database.size(); i+=9 )
	{
		const std::string &file_name = database[i];
		const std::string &name_tag = database[i+1];

		if ( file_name.empty() || file_name.find_last_of(".") == std::string::npos )
		{
			ROS_WARN("SimulationBackend::setupRigidObjectDatabase - %d'th element of graspable_objects has and invalid file_name specified (empty or no file extension), skipping...", (int)i);
			continue;
		}

		// Check if an object with the same name already exists, which is not allowed due to the save system
		std::string final_object_name = file_name.substr(0, file_name.find_last_of("."));
		if ( !name_tag.empty() && name_tag.compare(" ") != 0 )  // append name_tag if valid
			final_object_name += name_tag;

		for ( size_t j=0; j<rigid_object_database_.size(); ++j )
		{
			if ( rigid_object_database_[j].object_name.compare(final_object_name) == 0 )  // object name has been found -> not allowed
			{
				ROS_ERROR("SimulationBackend::setupRigidObjectDatabase - Two elements in graspable_objects database have the same internal object name %s which is not allowed!", final_object_name.c_str());
				return false;
			}
		}

		size_t camera_flight_index = 0;
		try
		{
			camera_flight_index = std::stoul(database[i+2]);

			if ( camera_flight_index >= active_camera_flights_.size() )
			{
				ROS_WARN("SimulatorEnvironment::setupRigidObjectDatabase - Invalid camera_flight_index in graspable_objects for object %s, skipping object.", file_name.c_str());
				continue;
			}
		}
		catch(...)
		{
			ROS_WARN("SimulatorEnvironment::setupRigidObjectDatabase - Corrupted camera_flight_index in graspable_objects for object %s, skipping object.", file_name.c_str());
			continue;
		}

		bool corrupted = false;
		double xyz_rpy[6];  // x, y, z, yaw, pitch, roll values
		for ( short j=0; j<6; ++j )
		{
			try
			{
				xyz_rpy[j] = std::stod(database[i+j+3]);  // skip entries before pose values -> +3
			}
			catch (...)
			{
				ROS_WARN("SimulatorEnvironment::setupRigidObjectDatabase - Corrupted entry %d in graspable_objects for object %s, skipping object.", (j+1), file_name.c_str());
				corrupted = true;
				break;
			}
		}

		if ( corrupted )  // skip this entry
			continue;

		RigidObjectDefaults element(file_name, final_object_name, camera_flight_index, xyz_rpy[0], xyz_rpy[1], xyz_rpy[2], xyz_rpy[3], xyz_rpy[4], xyz_rpy[5]);
		rigid_object_database_.push_back(element);
	}

	if ( rigid_object_database_.empty() )
	{
		ROS_ERROR("SimulatorEnvironment::setupRigidObjectDatabase - Generated rigid_object_database (created from graspable_objects) is empty, exiting.");
		return false;
	}

	return true;
}

bool SimulationBackend::isInitialized()
{
	return initialized_;
}


int SimulationBackend::runSimulation()
{
	while ( ros::ok() )
	{
		advanceSimulationState();
		simulation_->Advance(dt_);
	}

	return 0;
}

void SimulationBackend::advanceSimulationState()
{
	if ( !initialized_ )
	{
		ROS_WARN("SimulationBackend::advanceSimulationState - SimulationBackend has not been initialized yet!");
		return;
	}

	if ( simulation_->time > 0.5 )  // wait a bit so simulation and visualization can set up
	{
		switch ( simulation_state_ )
		{
			case ST_OBJECT_SPAWN_NEXT:
			{
				t_start_ = std::chrono::high_resolution_clock::now();  // DEBUGGING
				short spawn_result = spawnNextGraspableObject();

				if ( spawn_result == 0 )
				{
					ROS_WARN("Spawned object %d/%d: %s.", (int)rigid_object_database_counter_, (int)rigid_object_database_.size(), current_rigid_object_defaults_->object_name.c_str());
					last_object_movement_time_ = 0.0;
					simulation_state_ = ST_OBJECT_WAIT;
				}
				else if ( spawn_result == 1 )
					simulation_state_ = ST_FINISHED;
				else
					ROS_WARN("SimulationBackend::advanceSimulationState - Could not spawn object into scene.");

				break;
			}
			case ST_OBJECT_WAIT:  // wait for object movement to decline
			{
				if ( !isCurrentObjectMoving() )
				{
					if ( last_object_movement_time_ < 0.0001 )
					{
						last_object_movement_time_ = simulation_->time;
						break;
					}

					if ( simulation_->time - last_object_movement_time_ < 0.5 )  // wait a bit more
					{
						current_rigid_object_->SetVelocity(Math3D::Vector3(0,0,0), Math3D::Vector3(0,0,0));  // set velocities to 0
						break;
					}

					current_rigid_object_->GetTransform(original_object_pose_);  // save original transform of rigid object to check if gripper changed its pose after moving
					Math3D::AABB3D object_aabb;
					getCurrentBoundigBox(object_aabb);
					object_aabb.getMidpoint(original_object_aabb_center_);
					last_object_movement_time_ = 0.0;
					simulation_state_ = ST_CAMERA_SET_POSE;
				}
				else
					last_object_movement_time_ = simulation_->time;

				break;
			}
			case ST_CAMERA_SET_POSE:
				if ( setNextCameraPose() )
				{
					ROS_WARN("Camera pose %d/%d.", (int)camera_flight_counter_, (int)current_camera_flight_->size());
					simulation_state_ = ST_GRIPPER_MOVE_TO_START;
				}
				else
					simulation_state_ = ST_RESET_SIM;

				break;
			case ST_GRIPPER_MOVE_TO_START:  // set gripper back to start so that its geometry does not interfere with camera
				if ( !gripper_moves_ )
				{
					if ( needsEvasionManeuver(gripper_start_pose_) )  // check if gripper has to perform an evasion maneuver to avoid getting stuck in the ground
					{
						triggerEvasionManeuver();
						break;
					}

					prepareGripperMovement();
					gripper_interface_->setMoveToConfig(gripper_start_configuration_);
				}
				else if ( gripper_interface_->hasReachedTarget(gripper_start_configuration_) )
				{
					gripper_moves_ = false;
					gripper_interface_->stopMovement();
					resetCurrentRigidObject();

					if ( recover_to_perspective_augmentation_state_ )
						simulation_state_ = ST_PERSPECTIVE_AUGMENTATION;  // now that gripper is away from camera view, go back to perspective augmentation
					else
					{
						toggleCameraState(true);  // turn camera back on before switching to ST_GET_GRIPPERPOSES
						simulation_state_ = ST_GET_GRIPPERPOSES;
					}

				}
				else if ( hasGripperTimedOut() )  // ToDo: How to proceed now?
				{
					ROS_ERROR("SimulationBackend::advanceSimulationState - In ST_GRIPPER_MOVE_TO_START: Timeout!");
					move_start_errors_++;
					gripper_moves_ = false;
					gripper_interface_->stopMovement();
					simulation_state_ = ST_RESET_SIM;
				}

				break;
			case ST_GRIPPER_EVASION_MANEUVER:  // this state might be called from ST_GRIPPER_MOVE_TO_START or ST_GRIPPER_PREPARE_GRASP
				if ( !gripper_moves_ )
				{
					prepareGripperMovement();
					gripper_interface_->setTerrainEvasionMovement(gripper_evasion_target_, gripper_evasion_config_);
				}
				else if ( gripper_interface_->hasReachedTarget(gripper_evasion_config_) )
				{
					gripper_moves_ = false;
					gripper_interface_->stopMovement();
					simulation_state_ = pre_gripper_evasion_state_;  // go back to where we came from
				}
				else if ( hasGripperTimedOut() )
				{
					ROS_ERROR("SimulationBackend::advanceSimulationState - In ST_GRIPPER_EVASION_MANEUVER: Timeout!");
					evasion_errors_++;
					skip_evasion_once_ = true;  // do not call same evasion maneuver again
					gripper_moves_ = false;
					gripper_interface_->stopMovement();
					simulation_state_ = pre_gripper_evasion_state_;
				}

				break;
			case ST_GET_GRIPPERPOSES:
			{
				sensor_msgs::PointCloud2 current_pointcloud;
				getSegmentedPointCloud2AndDepthImage(current_pointcloud, current_depth_image_);

				// Set up the grasp pose service instance and feed it with the necessary values to compute the grasp poses
				ipa_grasppose_srvs::GetGraspPoses grasp_srv;
				grasp_srv.request.point_cloud = current_pointcloud;

				// Get the transform from camera frame (target frame in which everything will be transformed to) to grasp frame and store its quaternion to the request.
				// Since target frame == camera frame, store the trivial transformation.
				geometry_msgs::Pose target_to_camera_trafo;
				target_to_camera_trafo.orientation.w = 1;
				target_to_camera_trafo.orientation.x = 0;
				target_to_camera_trafo.orientation.y = 0;
				target_to_camera_trafo.orientation.z = 0;
				target_to_camera_trafo.position.x = 0;
				target_to_camera_trafo.position.y = 0;
				target_to_camera_trafo.position.z = 0;
				grasp_srv.request.target_to_camera_transform = target_to_camera_trafo;

				const Math3D::Vector3 grasp_direction_vector_grasp_frame = gripper_interface_->getGraspDirectionVector();
				grasp_srv.request.grasp_direction_vector_grasp_frame.x = grasp_direction_vector_grasp_frame.x;
				grasp_srv.request.grasp_direction_vector_grasp_frame.y = grasp_direction_vector_grasp_frame.y;
				grasp_srv.request.grasp_direction_vector_grasp_frame.z = grasp_direction_vector_grasp_frame.z;

				Math3D::Vector3 gripper_alignment_vector_grasp_frame = gripper_interface_->getGripperAlignmentVector();
				grasp_srv.request.gripper_alignment_vector_grasp_frame.x = gripper_alignment_vector_grasp_frame.x;
				grasp_srv.request.gripper_alignment_vector_grasp_frame.y = gripper_alignment_vector_grasp_frame.y;
				grasp_srv.request.gripper_alignment_vector_grasp_frame.z = gripper_alignment_vector_grasp_frame.z;

				// Set other settings
				grasp_srv.request.normal_search_radius = normal_search_radius_;
				grasp_srv.request.downsampling_voxel_leaf_size = downsampling_voxel_leaf_size_;

				if ( !sample_graspposes_client_.call(grasp_srv) )
				{
					ROS_WARN("SimulationBackend::advanceSimulationState - Failed to call sample_graspposes_srv service.");
					break;
				}

				// Filter grasp poses so that each can be approached by the gripper in a direct line
				// Store valid grasp poses afterwards
				gripper_grasp_poses_.clear();
				const Math3D::RigidTransform world_to_camera = camera_->links[camera_sensor_->link].T_World * camera_sensor_->Tsensor;
				for ( const geometry_msgs::Pose &grasp_pose : grasp_srv.response.grasp_poses_target_frame )
				{
					Math3D::QuaternionRotation grasp_quaternion(grasp_pose.orientation.w, grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z);
					Math3D::Matrix3 rotation_matrix;
					grasp_quaternion.getMatrix(rotation_matrix);
					Math3D::RigidTransform camera_to_grasp_pose(rotation_matrix, Math3D::Vector3(grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z));  // grasp pose in grasp frame coordinates
					Math3D::RigidTransform world_to_grasp_pose = world_to_camera * camera_to_grasp_pose;  // grasp pose in world coordinates
					Math3D::Vector3 candidate_world = world_to_grasp_pose.R * (gripper_interface_->getGraspDirectionVector() * -1);  // retrieve surface normals and its candidates in world coordinates for hemispherical filtering

					if ( !isInRelevantHemisphere(candidate_world, plane_normal_filter_world_) )  // filter out grasp poses that have an invalid approach direction (e.g. from below) -> can't be reached in a direct line
						continue;

					GripperGraspPose new_pose;
					new_pose.grasp_pose_msg_camera_frame.position.x = grasp_pose.position.x;  // Save original grasp pose data (relative to camera frame)
					new_pose.grasp_pose_msg_camera_frame.position.y = grasp_pose.position.y;
					new_pose.grasp_pose_msg_camera_frame.position.z = grasp_pose.position.z;
					new_pose.grasp_pose_msg_camera_frame.orientation.w = grasp_pose.orientation.w;
					new_pose.grasp_pose_msg_camera_frame.orientation.x = grasp_pose.orientation.x;
					new_pose.grasp_pose_msg_camera_frame.orientation.y = grasp_pose.orientation.y;
					new_pose.grasp_pose_msg_camera_frame.orientation.z = grasp_pose.orientation.z;
					new_pose.grasp_pose = world_to_grasp_pose;  // Save transformed grasp pose data (relative to world frame) which is used by the simulation
					new_pose.pre_grasp_pose = world_to_grasp_pose;
					Math3D::Vector3 pre_grasp_offset_grasp_pose = -std::abs(gripper_interface_->getPreGraspOffset()) * gripper_interface_->getGraspDirectionVector();  // shift along the gripper direction vector in grasp pose coordinates
					new_pose.pre_grasp_pose.t = world_to_grasp_pose * pre_grasp_offset_grasp_pose;  // apply offset

					gripper_grasp_poses_.push_back(new_pose);
				}

				ROS_WARN("SimulationBackend::advanceSimulationState - %d grasp poses remain after filtering.", (int)gripper_grasp_poses_.size());
				toggleCameraState(false);  // turn off camera for better preformance

				if ( !gripper_grasp_poses_.empty() )
					simulation_state_ = ST_GRIPPER_PREPARE_GRASP;
				else
					simulation_state_ = ST_RESET_SIM;
				break;
			}
			case ST_GRIPPER_PREPARE_GRASP:  // move to the pre grasp pose and open gripper

				// Object might have gotten stuck within gripper, so reset it to original location when gripper has some distance
				if ( error_flag_ && gripper_moves_ && hasObjectMovedTooMuch(OBJECT_DEVIATION_SOFT_POS, OBJECT_DEVIATION_SOFT_ROT) )
				{
					Math3D::RigidTransform world_to_grasp;
					gripper_interface_->getWorldToGraspFrame(world_to_grasp);
					double distance_to_reset = (original_object_aabb_center_ - world_to_grasp.t).norm();

					if ( distance_to_reset > 0.1 )  // 10 cm away -> reset
						resetCurrentRigidObject();
				}

				if ( !gripper_moves_ )
				{
					if ( !error_flag_ )  // normal call, not an emergency call to reset the gripper to previous pre grasp pose (from within ST_ERROR_STATE)
					{
						if ( gripper_poses_counter_ < gripper_grasp_poses_.size() )
						{
							current_grasp_pose_ = &gripper_grasp_poses_[gripper_poses_counter_];  // Assign current pose
							if ( needsEvasionManeuver(current_grasp_pose_->pre_grasp_pose) )  // check if gripper has to perform an evasion maneuver to avoid getting stuck in the ground
							{
								triggerEvasionManeuver();
								break;
							}

							gripper_poses_counter_++;  // increment for next call
							ROS_WARN("Trying grasp pose %d/%d.", (int)gripper_poses_counter_, (int)gripper_grasp_poses_.size());
						}
						else
						{
							ROS_WARN("SimulationBackend::advanceSimulationState - All gripper poses have already been checked.");
							simulation_state_ = ST_RESET_SIM;
							break;
						}
					}

					prepareGripperMovement();
					gripper_interface_->computeConfigFromCartesian(current_grasp_pose_->pre_grasp_pose, current_gripper_target_);  // build pre grasp...
					gripper_interface_->addOpenGripperToConfig(current_gripper_target_);  // ... and open gripper config
					gripper_interface_->setMoveToConfig(current_gripper_target_);  // finally move
				}
				else if ( gripper_interface_->hasReachedTarget(current_gripper_target_) )
				{
					gripper_moves_ = false;
					gripper_interface_->stopMovement();

					// Sometimes the gripper manipulates the object when the start location is on the opposite side
					resetCurrentRigidObject();

					if ( error_flag_ )
						simulation_state_ = ST_ERROR_STATE;  // go back to error state and resolve error
					else
						simulation_state_ = ST_GRIPPER_MOVE_TO_GRASP;
				}
				else if ( isGripperStuck() || hasGripperTimedOut() )
				{
					// This can't be recovered easily, where to move?
					ROS_ERROR("SimulationBackend::advanceSimulationState - Unable to approach to the current pre grasp position.");
					gripper_moves_ = false;
					gripper_interface_->stopMovement();
					simulation_state_ = ST_RESET_SIM;  // try to get to other locations
					// DEBUGGING
					prepare_errors_++;
					std::cout << "ERROR CASE: " << error_flag_ << std::endl;
					std::cout << simulation_->time << "/" << movement_timeout_time_ << "/" << stuck_counter_ << std::endl;
					std::cout << "HAS CONTACT: " << gripper_interface_->hasContactWithRigidObject() << std::endl;
					const std::string file_name = "error_"+current_rigid_object_->obj.name+"_"+std::to_string(prepare_errors_);
					GLSaveScreenshot((file_name+"_screenshot.ppm").c_str());
					// End debugging
				}
				break;
			case ST_GRIPPER_MOVE_TO_GRASP:  // move to the actual grasp pose
				if ( !gripper_moves_ )
				{
					prepareGripperMovement();
					gripper_interface_->setMoveToCartesian(current_grasp_pose_->grasp_pose, current_gripper_target_,
														   gripper_interface_->getObjectApproachingMoveTime(), &gui_backend_);
				}
				else if ( hasObjectMovedTooMuch(OBJECT_DEVIATION_STRICT_POS, OBJECT_DEVIATION_STRICT_ROT) )  // check if gripper movement violates object's pose too much
					triggerErrorState(false, "SimulationBackend::advanceSimulationState - in ST_GRIPPER_MOVE_TO_GRASP: Object's pose has been altered too much.");
				else if ( gripper_interface_->hasReachedTarget(current_gripper_target_) )
				{
					gripper_moves_ = false;
					gripper_interface_->stopMovement();
					gripper_interface_->getWorldToGraspFrame(gripper_at_object_pose_);  // save pose when having arrive at the object to check against it later on while closing fingers
					simulation_state_ = ST_GRIPPER_CLOSING;
				}
				else if ( isGripperCollidingWithEnvironment() || hasGripperTimedOut() )
					triggerErrorState(true, "SimulationBackend::advanceSimulationState - in ST_GRIPPER_MOVE_TO_GRASP.");
				else if ( isGripperStuck() )
					triggerErrorState(false, "SimulationBackend::advanceSimulationState - in ST_GRIPPER_MOVE_TO_GRASP.");

				break;
			case ST_GRIPPER_CLOSING:
			{
				if ( !gripper_moves_ )
					prepareGripperMovement();

				gripper_interface_->stepCloseGripper(current_gripper_target_);  // gradually close gripper while keeping it at target configuration

				// Check that object's pose and gripper's at object pose do not change too much
				Math3D::RigidTransform current_gripper_pose;
				gripper_interface_->getWorldToGraspFrame(current_gripper_pose);

				if ( hasObjectMovedTooMuch(OBJECT_DEVIATION_SOFT_POS, OBJECT_DEVIATION_SOFT_ROT) )  // object should not have diverged much from its original pose
					triggerErrorState(false, "SimulationBackend::advanceSimulationState - in ST_GRIPPER_CLOSING: Object's pose has been altered too much.");
				else if ( !CalcUtils::equalTransforms(gripper_at_object_pose_, current_gripper_pose, GRIPPER_DEVIATION_POS, GRIPPER_DEVIATION_ROT) )  // gripper should not have diverged from original pose
					triggerErrorState(false, "SimulationBackend::advanceSimulationState - in ST_GRIPPER_CLOSING: Gripper's pose has been altered too much.");
				else if ( gripper_interface_->isGripperClosed() )
				{
					gripper_moves_ = false;
					gripper_interface_->stopMovement();

					// Save the grasp frame location in object coordinates to verify the grasp slippage later on
					Math3D::RigidTransform object_to_world;
					current_rigid_object_->GetTransform(object_to_world);  // from world to object
					object_to_world.inplaceInverse();  // invert trafo: object to world
					Math3D::RigidTransform world_to_grasp_frame;
					gripper_interface_->getWorldToGraspFrame(world_to_grasp_frame);
					const Math3D::RigidTransform object_to_grasp_frame = object_to_world * world_to_grasp_frame;
					object_to_grasp_frame_ = object_to_grasp_frame.t;  // save vector in object coordinates

					simulation_state_ = ST_GRIPPER_LIFT_OBJECT;
				}
				else if ( isGripperCollidingWithEnvironment() || hasGripperTimedOut() )
					triggerErrorState(true, "SimulationBackend::advanceSimulationState - in ST_GRIPPER_CLOSING.");

				break;
			}
			case ST_GRIPPER_LIFT_OBJECT:
				if ( !gripper_moves_ )
				{
					prepareGripperMovement();
					gripper_interface_->setGraspValidationMovement(current_gripper_target_);
				}
				else if ( gripper_interface_->hasReachedTarget(current_gripper_target_, true) || hasGripperLostObject(OBJECT_DEVIATION_SOFT_POS) )
				{
					gripper_moves_ = false;
					gripper_interface_->stopMovement();
					simulation_state_ = ST_ASSESS_GRASP;
				}
				else if ( isGripperCollidingWithEnvironment() || isGripperStuck() )
					triggerErrorState(true, "SimulationBackend::advanceSimulationState - in ST_GRIPPER_LIFT_OBJECT.");

				break;
			case ST_ERROR_STATE:
				if ( !error_flag_ )
				{
					error_flag_ = true;
					simulation_state_ = ST_GRIPPER_PREPARE_GRASP;  // move gripper back to where it came from
				}
				else  // will be called the 2nd time this state is entered
					simulation_state_ = ST_ASSESS_GRASP;
				break;
			case ST_ASSESS_GRASP:
			{
				if ( grasp_assessment_time_ < 0.001 )
					grasp_assessment_time_ = simulation_->time;

				if ( !ignore_last_grasp_ )
				{
					bool result = false;
					if ( !error_flag_ )
					{
						gripper_interface_->setMoveToConfig(current_gripper_target_);  // keep the pressure active
						if ( simulation_->time - grasp_assessment_time_ < 2.0 )  // wait some time before assessment so that object movement can settle
							break;

						result = assessGrasp();

						if ( result )
							ROS_WARN("Good grasp found!");
					}

					// DEBUGGING
					valid_grasps_++;
					if ( result )
						successful_grasps_++;
					// End debugging
					storeGrasp(result);
				}

				all_grasps_++;  // DEBUGGING
				simulation_state_ = ST_RESET_SIM;
				break;
			}
			case ST_RESET_SIM:
				resetCurrentRigidObject();
				error_flag_ = false;
				ignore_last_grasp_ = false;
				grasp_assessment_time_ = 0.0;

				// DEBUGGING
				/*if ( gripper_poses_counter_ % 10 == 0 )
				{
					std::chrono::high_resolution_clock::time_point t_now = std::chrono::high_resolution_clock::now();
					auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_now - t_start_).count();
					std::cout << std::endl << "[" << camera_flight_counter_ << "/" << current_camera_flight_->size()
							  << "] RUNTIME: " << duration/(1000*3600.f) << " hours." << std::endl;
					std::cout << "GRASPS: e" << prepare_errors_ << "/s" << successful_grasps_ << "/v" << valid_grasps_ << "/a" << all_grasps_ << std::endl;
					std::cout << "EVASIONS: e" << evasion_errors_ << "/st" << start_evasions_ << "/en" << end_evasions_ << std::endl;
					std::cout << "START ERRORS: " << move_start_errors_ << std::endl;
					std::cout << "TOTAL GRASPS: " << total_grasp_poses_ << std::endl;
					std::cout << "FAILED SAVE CALLS: " << failed_save_calls_ << std::endl;
					std::cout << std::endl;
				}*/
				// End debugging

				if ( gripper_poses_counter_ >= gripper_grasp_poses_.size() )  // All gripper poses have been reached for the current camera pose
				{
					// Call save node service here with all successfully tested grasp poses of one camera view angle
					if ( !evaluated_grasps_.empty() )  // Do not call when there is nothing to be saved
					{
						if ( current_delta_poses_set_ != nullptr )  // now create new data using perspective augmentation and save it then...
						{
							simulation_state_ = ST_PERSPECTIVE_AUGMENTATION;
							break;
						}

						const size_t actual_view_counter = camera_flight_counter_+saved_perspective_augmentations_;
						if ( !callSaveGraspdataService(current_rigid_object_->obj.name, actual_view_counter, current_depth_image_, evaluated_grasps_) )  // ...save originally evaluated data
							break;
					}

					// DEBUGGING
					/*simulation_state_ = ST_FINISHED;
					break;*/
					// End debugging

					evaluated_grasps_.clear();
					gripper_grasp_poses_.clear();
					gripper_poses_counter_ = 0;
					saveSimulationDataToDrive();  // Save some simulation settings to drive for recovery after crash

					if ( camera_flight_counter_ >= current_camera_flight_->size() )  // All camera positions have been visited for this object
					{
						camera_flight_counter_ = 0;
						saved_perspective_augmentations_ = 0;  // reset additinally saved perspectives counter for each new object

						if ( rigid_object_database_counter_ >= rigid_object_database_.size() )
						{
							FileUtils::deleteFile(simulation_storage_file_path_);  // Delete save file
							simulation_state_ = ST_FINISHED;
						}
						else
						{
							disposeCurrentRigidObject();
							simulation_state_ = ST_OBJECT_SPAWN_NEXT;
						}
					}
					else
						simulation_state_ = ST_CAMERA_SET_POSE;
				}
				else
					simulation_state_ = ST_GRIPPER_PREPARE_GRASP;

				break;
			case ST_PERSPECTIVE_AUGMENTATION:
				if ( !perspective_augmentation_prepared_ )
				{
					recover_to_perspective_augmentation_state_ = true;  // recover after finishing ST_GRIPPER_MOVE_TO_START
					if ( !gripper_interface_->hasReachedTarget(gripper_start_configuration_) )  // send gripper to start before taking further depth images
					{
						simulation_state_ = ST_GRIPPER_MOVE_TO_START;
						break;
					}

					recover_to_perspective_augmentation_state_ = false;
					perspective_augmentation_prepared_ = true;
					toggleCameraState(true);
					perspective_augmentation_counter_ = 0;  // reset counter
					original_camera_pose_ = camera_sensor_->Tsensor;  // save original camera pose to vary around using delta poses
					setNextAugmentedPerspective();
					break;
				}
				else
				{
					std::vector<Math3D::Vector3> point_cloud;
					sensor_msgs::Image depth_image;
					getSegmentedPointCloudAndDepthImage(point_cloud, depth_image);  // capture new depth image

					// Transform old grasp poses into new camera frame
					std::vector<GripperGraspPose> grasps_transformed;  // transformed grasp poses (only the pose message, rest is the same)
					std::vector<EvaluatedGrasp> evaluated_grasps_transformed;  // temporary containter of transformed pose message
					grasps_transformed.reserve(evaluated_grasps_.size());
					evaluated_grasps_transformed.reserve(evaluated_grasps_.size());
					Math3D::RigidTransform camera_to_world;  // current camera to world transform
					camera_->links[camera_sensor_->link].T_World.getInverse(camera_to_world);
					Math3D::RigidTransform sensor_to_camera;
					camera_sensor_->Tsensor.getInverse(sensor_to_camera);
					const Math3D::RigidTransform sensor_to_world = sensor_to_camera * camera_to_world;
					for ( const EvaluatedGrasp &grasp : evaluated_grasps_ )
					{
						const Math3D::RigidTransform grasp_pose_new_camera = sensor_to_world * grasp.grasp_pose->grasp_pose;  // transfrom grasp pose from world to camera sensor frame

						// Drop grasp poses that do not have pointcloud points around
						if ( !isGraspPoseWithinPointCloud(grasp_pose_new_camera, point_cloud) )
							continue;

						GripperGraspPose grasp_transformed;
						CalcUtils::convertToPose(grasp_pose_new_camera, grasp_transformed.grasp_pose_msg_camera_frame);
						grasps_transformed.push_back(grasp_transformed);

						EvaluatedGrasp evaluated_grasp_transformed;
						evaluated_grasp_transformed.grasp_pose = &grasps_transformed.back();  // pointer to last added element
						evaluated_grasp_transformed.rating = grasp.rating;
						evaluated_grasps_transformed.push_back(evaluated_grasp_transformed);
					}

					// Call callSaveGraspdataService again with new perspective
					if ( !evaluated_grasps_transformed.empty() )
					{
						const size_t actual_view_counter = camera_flight_counter_+saved_perspective_augmentations_;
						callSaveGraspdataService(current_rigid_object_->obj.name, actual_view_counter, depth_image, evaluated_grasps_transformed);
						saved_perspective_augmentations_++;
					}

					// Set next perspective
					if ( perspective_augmentation_counter_ < current_delta_poses_set_->size() )
						setNextAugmentedPerspective();
					else
					{
						perspective_augmentation_prepared_ = false;  // reset for next camera flight pose
						toggleCameraState(false);
						current_delta_poses_set_ = nullptr;  // deactivate perspective augmentation for this camera flight pose so it won't be re-called when re-entering ST_RESET_SIM again
						simulation_state_ = ST_RESET_SIM;  // go back to reset state and perform the remaining tasks
					}
				}
				break;
			case ST_FINISHED:
				if ( !simulation_has_finished_ )
				{
					simulation_has_finished_ = true;
					ROS_WARN("SIMULATION HAS FINISHED");

					// DEBUGGING
					std::chrono::high_resolution_clock::time_point t_now = std::chrono::high_resolution_clock::now();
					auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_now - t_start_).count();
					std::cout << std::endl << "[" << camera_flight_counter_ << "/" << current_camera_flight_->size()
							  << "] RUNTIME: " << duration/(1000*3600.f) << " hours." << std::endl;
					std::cout << "GRASPS: e" << prepare_errors_ << "/s" << successful_grasps_ << "/v" << valid_grasps_ << "/a" << all_grasps_ << std::endl;
					std::cout << "EVASIONS: e" << evasion_errors_ << "/st" << start_evasions_ << "/en" << end_evasions_ << std::endl;
					std::cout << "START ERRORS: " << move_start_errors_ << std::endl;
					std::cout << "TOTAL GRASPS: " << total_grasp_poses_ << std::endl;
					std::cout << "FAILED SAVE CALLS: " << failed_save_calls_ << std::endl;
					std::cout << std::endl;
					// End debugging
				}
				break;
		}
	}
}

short SimulationBackend::spawnNextGraspableObject()
{
	if ( rigid_object_database_counter_ >= rigid_object_database_.size() )
	{
		ROS_WARN("SimulationBackend::spawnNextGraspableObject - All objects have already been spawned.");
		return 1;
	}

	current_rigid_object_defaults_ = &rigid_object_database_[rigid_object_database_counter_++];
	std::string file = rigid_objects_path_+current_rigid_object_defaults_->file_name;

	int object_id = world_->LoadElement(file.c_str());
	if( object_id < 0 )
	{
		ROS_ERROR("SimulatorEnvironment::spawnGraspableObject - Error loading object %s.", file.c_str());
		return -1;
	}
	else
	{
		size_t object_idx = world_->IsRigidObject(object_id);
		if ( object_idx < 0 )
		{
			ROS_ERROR("SimulatorEnvironment::spawnGraspableObject - %s is not a rigid object.", file.c_str());
			return -1;
		}

		RigidObject* object = world_->rigidObjects[object_idx].get();
		object->name = current_rigid_object_defaults_->object_name;

		if ( !setInitialTransformOfRigidObject(*object, *current_rigid_object_defaults_) )
		{
			ROS_ERROR("SimulatorEnvironment::spawnNextGraspableObject - Could not set initial transform of %s, deleting object.", current_rigid_object_defaults_->object_name.c_str());
			world_->DeleteRigidObject(object->name);
			current_rigid_object_ = nullptr;
			return -1;
		}
		else  // update ode about new object and update camera flight
		{
			simulation_->odesim.AddObject(*object);
			current_rigid_object_ = simulation_->odesim.object(simulation_->odesim.numObjects()-1);
			simulation_->odesim.ClearContactFeedback();  // remove deprecated contact feedback pairs (of disposed objects)
			simulation_->contactFeedback.clear();
			gui_backend_.InitContactFeedbackAll();
			current_camera_flight_ = &active_camera_flights_[current_rigid_object_defaults_->camera_flight_index];
		}
	}

	return 0;
}

// Set the location of a rigid object in the simulated world
bool SimulationBackend::setInitialTransformOfRigidObject(const RigidObject &object, const RigidObjectDefaults &default_values)
{
	int obj_id = world_->GetID(object.name);
	if ( obj_id < 0 )
	{
		ROS_ERROR("SimulatorEnvironment::transformRigidObject - Can not find %s in world to transform it", object.name.c_str());
		return false;
	}

	Math3D::Matrix3 rotation_mat;
	default_values.rotation.getMatrixZYX(rotation_mat);  // rotation definition as in TF: first around z, then y' and finally roll around x''
	Math3D::RigidTransform trafo_mat(rotation_mat, default_values.position);
	world_->SetTransform(obj_id, trafo_mat);
	return true;
}

// Set camera to new pose
bool SimulationBackend::setNextCameraPose()
{
	if ( camera_flight_counter_ >= current_camera_flight_->size() )
	{
		ROS_WARN("SimulationBackend::setCameraPose - All camera poses have already been visited.");
		return false;
	}

	Math3D::AABB3D aabb;
	if ( !getCurrentBoundigBox(aabb) )
	{
		ROS_WARN("SimulationBackend::setCameraPose - Could not get bounding box of current object.");
		return false;
	}

	// Calculate the absolute camera flight position in camera coordinates
	const Math3D::RigidTransform &next_pose = (*current_camera_flight_)[camera_flight_counter_++];
	Math3D::RigidTransform camera_to_world;
	camera_->links[camera_sensor_->link].T_World.getInverse(camera_to_world);
	Math3D::RigidTransform world_to_target;
	world_to_target.t = original_object_aabb_center_ + next_pose.t;
	world_to_target.R.setIdentity();
	camera_sensor_->Tsensor = camera_to_world * world_to_target;
	camera_sensor_->Tsensor.R = next_pose.R;  // Rotations are already final in camera coordinates

	// Assign the current delta poses that correspond to the current radius
	const double radius = next_pose.t.norm();
	current_delta_poses_set_ = nullptr;  // rest it and try to find the matching one
	for ( DeltaPosesSet &set : perspective_augmentation_ )
	{
		if ( std::abs(set.radius - radius) < 0.00001 )  // found
		{
			current_delta_poses_set_ = &(set.delta_poses);
			break;
		}
	}

	return true;
}

bool SimulationBackend::isInRelevantHemisphere(const Math3D::Vector3 &candidate, const Math3D::Vector3 &hemisphere_direction)
{	
	return hemisphere_direction.isZero() || hemisphere_direction.dot(candidate) > 0;  // zero means that candidate is always relevant
}

bool SimulationBackend::isCurrentObjectMoving(const double &angular_theshold, const double &linear_threshold)
{
	Math3D::Vector3 angular_vel, linear_vel;
	current_rigid_object_->GetVelocity(angular_vel, linear_vel);

	if ( angular_vel.norm() > angular_theshold || linear_vel.norm() > linear_threshold )  // object is still moving
		return true;
	else
		return false;
}

void SimulationBackend::prepareGripperMovement()
{
	gripper_moves_ = true;
	last_stamped_gripper_config_.time_stamp = 0.0;  // force update of last stamped gripper congiguration in stuck detection
	stuck_counter_ = 0;
	movement_timeout_time_ = simulation_->time + gripper_interface_->getGripperMovementTimeout(simulation_state_);
}

void SimulationBackend::triggerErrorState(const bool &ignore_grasp, const std::string &error_msg)
{
	gripper_moves_ = false;
	ignore_last_grasp_ = ignore_grasp;
	gripper_interface_->stopMovement();
	simulation_state_ = ST_ERROR_STATE;

	if ( !error_msg.empty() )
		ROS_WARN("%s", error_msg.c_str());
}

void SimulationBackend::getSegmentedPointCloud2AndDepthImage(sensor_msgs::PointCloud2 &segmented_point_cloud2, sensor_msgs::Image &segmented_depth_image)
{
	if ( pcl_conversion_.use_boundingbox_filter )  // set up bounding box filter needed for point cloud segmentation
	{
		Math3D::RigidTransform camera_to_world = camera_->links[camera_sensor_->link].T_World * camera_sensor_->Tsensor;
		camera_to_world.inplaceInverse();
		Math3D::AABB3D aabb_world;
		getCurrentBoundigBox(aabb_world);
		pcl_conversion_.AABB_cam.setTransform(aabb_world, camera_to_world);
	}

	std::vector<double> measurements;
	const ros::Time current_time = ros::Time::now();
	const std::string camera_frame = "Klampt_camera";

	segmented_point_cloud2.header.stamp = current_time;
	segmented_point_cloud2.header.frame_id = camera_frame;
	segmented_depth_image.header.stamp = current_time;
	segmented_depth_image.header.frame_id = camera_frame;

	camera_sensor_->GetMeasurements(measurements);
	std::vector<size_t> current_object_pixels;
	CalcUtils::convertToFilteredPointCloud2(measurements, camera_sensor_->yres, camera_sensor_->xres, pcl_conversion_, segmented_point_cloud2, current_object_pixels);
	CalcUtils::convertToDepthImage(measurements, camera_sensor_->yres, camera_sensor_->xres, camera_sensor_->zmax, segmented_depth_image, current_object_pixels);

	// DEBUGGING
	point_cloud_2_pub.publish(segmented_point_cloud2);
	depth_image_pub.publish(segmented_depth_image);
	// End debugging
}

void SimulationBackend::getSegmentedPointCloudAndDepthImage(std::vector<Math3D::Vector3> &segmented_point_cloud, sensor_msgs::Image &segmented_depth_image)
{
	if ( pcl_conversion_.use_boundingbox_filter )  // set up bounding box filter needed for point cloud segmentation
	{
		Math3D::RigidTransform camera_to_world = camera_->links[camera_sensor_->link].T_World * camera_sensor_->Tsensor;
		camera_to_world.inplaceInverse();
		Math3D::AABB3D aabb_world;
		getCurrentBoundigBox(aabb_world);
		pcl_conversion_.AABB_cam.setTransform(aabb_world, camera_to_world);
	}

	std::vector<double> measurements;
	const ros::Time current_time = ros::Time::now();
	const std::string camera_frame = "Klampt_camera";

	segmented_depth_image.header.stamp = current_time;
	segmented_depth_image.header.frame_id = camera_frame;

	camera_sensor_->GetMeasurements(measurements);
	std::vector<size_t> current_object_pixels;
	CalcUtils::convertToFilteredPointCloud(measurements, camera_sensor_->yres, camera_sensor_->xres, pcl_conversion_, segmented_point_cloud, current_object_pixels);
	CalcUtils::convertToDepthImage(measurements, camera_sensor_->yres, camera_sensor_->xres, camera_sensor_->zmax, segmented_depth_image, current_object_pixels);
}

bool SimulationBackend::assessGrasp()
{
	bool still = !isCurrentObjectMoving(OBJECT_VELOCITY_ANGULAR, OBJECT_VELOCITY_LINEAR);

	if ( still )
		return (!hasGripperLostObject(OBJECT_DEVIATION_SOFT_POS) && hasObjectMovedTooMuch(OBJECT_DEVIATION_SOFT_POS, OBJECT_DEVIATION_SOFT_ROT));  // object should not have diverged much from its original pose, 5 cm deviation allowed

	return false;
}

bool SimulationBackend::hasGripperLostObject(const double &allowed_position_deviation)
{
	Math3D::RigidTransform current_object_trafo;
	current_rigid_object_->GetTransform(current_object_trafo);
	Math3D::RigidTransform current_grasp_frame_trafo;
	gripper_interface_->getWorldToGraspFrame(current_grasp_frame_trafo);

	const Math3D::Vector3 world_to_grasp_frame_original = current_grasp_frame_trafo.t;
	const Math3D::Vector3 world_to_grasp_frame_over_object = current_object_trafo * object_to_grasp_frame_;

	return (world_to_grasp_frame_original - world_to_grasp_frame_over_object).norm() > allowed_position_deviation;
}

void SimulationBackend::storeGrasp(const bool &good_grasp)
{
	EvaluatedGrasp new_grasp_evaluation;
	new_grasp_evaluation.grasp_pose = current_grasp_pose_;  // store only a reference as we already have this data for the lifetime of the instance
	new_grasp_evaluation.rating = good_grasp;
	evaluated_grasps_.push_back(new_grasp_evaluation);
}

void SimulationBackend::resetCurrentRigidObject()
{
	current_rigid_object_->SetTransform(original_object_pose_);
	current_rigid_object_->SetVelocity(Math3D::Vector3(0,0,0), Math3D::Vector3(0,0,0));
	simulation_->UpdateModel();
}


void SimulationBackend::disposeCurrentRigidObject()
{
	world_->DeleteRigidObject(current_rigid_object_->obj.name);
	simulation_->odesim.RemoveObject(current_rigid_object_);
	current_rigid_object_ = nullptr;
	simulation_->UpdateModel();
}

RobotWorld* SimulationBackend::getWorld()
{
	return world_;
}

WorldSimulation* SimulationBackend::getSimulation()
{
	return simulation_;
}

double SimulationBackend::getDeltaTime()
{
	return dt_;
}

shared_ptr<Robot> SimulationBackend::getCamera()
{
	return camera_;
}

CameraSensor* SimulationBackend::getCameraSensor()
{
	return camera_sensor_;
}

Math3D::Matrix4 SimulationBackend::getWorldToGripperGraspFrame()  // used to draw grasp frame
{
	Math3D::Matrix4 result;
	result.setZero();

	if ( gripper_interface_ != nullptr )
	{
		Math3D::RigidTransform trafo;
		gripper_interface_->getWorldToGraspFrame(trafo);
		result = (Math3D::Matrix4)trafo;
	}

	return result;
}

bool SimulationBackend::getCurrentBoundigBox(Math3D::AABB3D &bounding_box)
{
	if ( current_rigid_object_ == nullptr || current_rigid_object_->obj.geometry.Empty() )
		return false;

	bounding_box = current_rigid_object_->obj.geometry->GetAABB();
	return true;
}

Math3D::Matrix4 SimulationBackend::getWorldToCurrentGraspPose()  // used to draw grasp pose
{
	Math3D::Matrix4 result;
	result.setZero();

	if ( current_grasp_pose_ != nullptr )
		result = (Math3D::Matrix4)current_grasp_pose_->grasp_pose;

	return result;
}

// DEBUGGING
Math3D::Matrix4 SimulationBackend::getWorldToCurrentPreGraspPose()  // used to draw pre grasp pose
{
	Math3D::Matrix4 result;
	result.setZero();

	if ( current_grasp_pose_ != nullptr )
		result = (Math3D::Matrix4)current_grasp_pose_->pre_grasp_pose;

	return result;
}

void SimulationBackend::getAllCameraPoses(std::vector<Math3D::Matrix4> &result)
{
	if ( current_camera_flight_ == nullptr )
		return;

	const Math3D::RigidTransform &world_to_camera = camera_->links[camera_sensor_->link].T_World;
	result.reserve(current_camera_flight_->size());
	for ( const Math3D::RigidTransform &next_pose : *current_camera_flight_ )
	{
		Math3D::RigidTransform world_to_pose;
		world_to_pose.t = original_object_aabb_center_ + next_pose.t;
		world_to_pose.R = world_to_camera.R * next_pose.R;
		result.push_back((Math3D::Matrix4)world_to_pose);
	}
}

Math3D::Vector3 SimulationBackend::getOriginalAABBCenter()
{
	return original_object_aabb_center_;
}
// End debugging

void SimulationBackend::getWorldToGraspPoses(std::vector<Math3D::Matrix4> &result)
{
	result.reserve(gripper_grasp_poses_.size());
	for ( GripperGraspPose pose : gripper_grasp_poses_ )
		result.push_back((Math3D::Matrix4)pose.grasp_pose);
}

bool SimulationBackend::drawGripperGraspFrame()
{
	return draw_gripper_graspframe_;
}

bool SimulationBackend::drawAllGraspPoses()
{
	return draw_all_graspposes_;
}

bool SimulationBackend::drawCurrentGraspPose()
{
	return draw_current_grasppose_;
}

bool SimulationBackend::drawBoundingBox()
{
	return draw_bounding_box_;
}

bool SimulationBackend::drawCamera()
{
	return draw_camera_;
}

bool SimulationBackend::drawCameraFrustum()
{
	return draw_camera_frustum_;
}

bool SimulationBackend::simulationHasFinished()
{
	return simulation_has_finished_;
}

bool SimulationBackend::isGripperCollidingWithEnvironment()
{
	std::vector<int> robot_id(1, gripper_interface_->getGripperID());
	std::vector<int> env_ids;
	for ( size_t i=0; i<world_->terrains.size(); ++i )  // check collision against terrain geometry only (excluding rigid objects)
		env_ids.push_back(world_->TerrainID(i));

	std::pair<int, int> result = collision_detection_->CheckCollision(*world_, robot_id, env_ids, 0.01);  // 1 cm additional margin

	if ( result.first >= 0 )
	{
		ROS_WARN("SimulationBackend::isGripperCollidingWithEnvironment - Gripper is in collision with its environment.");
		return true;
	}

	return false;
}

bool SimulationBackend::hasGripperTimedOut()
{
	if ( movement_timeout_time_ > 0.0001 && simulation_->time > movement_timeout_time_ )
	{
		ROS_WARN("SimulationBackend::isGripperStuck - Current gripper movement has timed out.");
		movement_timeout_time_ = 0.0;
		stuck_counter_ = 0;
		return true;
	}

	return false;
}

bool SimulationBackend::isGripperStuck()
{
	if ( last_stamped_gripper_config_.time_stamp < 0.0001 )  // very first call
	{
		last_stuck_check_time_ = simulation_->time;
		last_stamped_gripper_config_.time_stamp = simulation_->time;
		last_stamped_gripper_config_.configuration = gripper_interface_->getCurrentConfig();
		return false;
	}

	const double check_inverval = dt_*STUCK_DETECTION_SAMPLING_FACTOR;
	if ( (simulation_->time - last_stuck_check_time_) > check_inverval )
	{
		// Check if contact force is too high
		bool stuck = gripper_interface_->maxContactForceExceeded();

		// Check if no movement
		if ( !stuck )
		{
			stuck = CalcUtils::equalConfigs(gripper_interface_->getCurrentConfig(), last_stamped_gripper_config_.configuration,
											gripper_interface_->getPerJointAccuracyParameters(), gripper_interface_->getRevoluteJointIndexes()); // approx. exactly equal configs -> approx no movement
			stuck &= gripper_interface_->hasContactWithRigidObject();  // combine it with whether gripper has contact. That way we exclude cases where gripper converges slowly to target pose without any contact being invloved
		}
		stuck_counter_ = stuck ? stuck_counter_+1 : 0;

		// Check if stuck condition applies
		if ( stuck_counter_ >= STUCK_COUNT_THRESHOLD )
		{
			ROS_WARN("SimulationBackend::isGripperStuck - Gripper is stuck.");
			movement_timeout_time_ = 0.0;
			stuck_counter_ = 0;
			return true;
		}

		// Update last stamped gripper config with current config
		if ( (simulation_->time - last_stamped_gripper_config_.time_stamp) > (check_inverval*STUCK_COUNT_THRESHOLD) )
		{
			last_stamped_gripper_config_.time_stamp = simulation_->time;
			last_stamped_gripper_config_.configuration = gripper_interface_->getCurrentConfig();
		}
	}

	return false;
}

bool SimulationBackend::hasObjectMovedTooMuch(const double &allowed_position_deviation, const double &allowed_rotation_deviation_angle)
{
	Math3D::RigidTransform current_object_pose;
	current_rigid_object_->GetTransform(current_object_pose);
	return !CalcUtils::equalTransforms(original_object_pose_, current_object_pose, allowed_position_deviation, allowed_rotation_deviation_angle);
}

bool SimulationBackend::loadSimulationDataFromDrive()
{
	std::string content = "";
	if ( FileUtils::readFileContent(simulation_storage_file_path_, content) )
	{
		try  // always check when loading from file, as it can be corrupted
		{
			std::vector<std::string> lines = FileUtils::string_split(content, "\n");
			if ( lines.size() == 3 )
			{
				const size_t zero = 0;
				rigid_object_database_counter_ = std::max( std::min( std::stoul(lines[0]), rigid_object_database_.size() ), zero );

				const size_t &flight_idx = rigid_object_database_[rigid_object_database_counter_].camera_flight_index;
				if ( flight_idx >= active_camera_flights_.size() )  // check if index is valid
				{
					ROS_ERROR("SimulatorEnvironment::loadSimulationDataFromDrive - %s has invalid content.", simulation_storage_file_path_.c_str());
					return false;
				}

				camera_flight_counter_ = std::max( std::min( std::stoul(lines[1]), active_camera_flights_[flight_idx].size()-1 ), zero );
				saved_perspective_augmentations_ = std::max( std::stoul(lines[2]), zero );
			}
			else
			{
				ROS_ERROR("SimulatorEnvironment::loadSimulationDataFromDrive - %s has corrupted content.", simulation_storage_file_path_.c_str());
				return false;
			}
		}
		catch (...)
		{
			ROS_ERROR("SimulatorEnvironment::loadSimulationDataFromDrive - Unable to read content of %s.", simulation_storage_file_path_.c_str());
			rigid_object_database_counter_ = 0;
			return false;
		}
	}

	return true;
}

bool SimulationBackend::saveSimulationDataToDrive()
{
	const bool new_object_inbound = camera_flight_counter_ >= current_camera_flight_->size();  // take reset of camera_flight_counter_ into account when a new object is about to spawn
	const std::string current_object_index_str = new_object_inbound ? std::to_string(rigid_object_database_counter_) : std::to_string(rigid_object_database_counter_-1);
	const std::string current_flight_index_str = new_object_inbound ? "0" : std::to_string(camera_flight_counter_);
	const std::string current_saved_perspective_augmentations_str = new_object_inbound ? "0" : std::to_string(saved_perspective_augmentations_);
	const std::string content = current_object_index_str+"\n"+current_flight_index_str+"\n"+current_saved_perspective_augmentations_str;  // save this to drive

	for ( unsigned short i=0; i<3; ++i )  // try up to 3 times to save
	{
		if ( FileUtils::writeFileContent(simulation_storage_file_path_, content) )
			return true;

		ros::Duration(0.25).sleep();  // wait a bit before next try
	}

	ROS_ERROR("SimulatorEnvironment::saveSimulationDataToDrive - Could not save current save current simulation data.");
	return false;
}

bool SimulationBackend::needsEvasionManeuver(const Math3D::RigidTransform &target_pose)
{
	if ( skip_evasion_once_ )  // skip maneuver once and reset flag
	{
		skip_evasion_once_ = false;
		return false;
	}

	Math3D::RigidTransform world_to_grasp_frame;
	gripper_interface_->getWorldToGraspFrame(world_to_grasp_frame);
	const Math3D::Vector3 &current_position = world_to_grasp_frame.t;
	Math3D::Vector3 obj_to_current_norm = (current_position - original_object_aabb_center_);
	obj_to_current_norm.inplaceNormalize();  // take normalization here so we can perform an angle dependent dot product later on
	Math3D::Vector3 obj_to_target_norm = (target_pose.t - original_object_aabb_center_);
	obj_to_target_norm.inplaceNormalize();  // also normalized -> result of dot product of two normalized vectors is related to the enclosing angle

	// Check wether angle between current position and target position is in a range that is not prone to terrain collision. E.g. moving 180° means switching sides which is prone to collision with ground terrain.
	const bool wide_angle = (Math3D::dot(obj_to_current_norm, obj_to_target_norm) < 0.6);  // 0.6 ~ 53.13 degrees
	if ( wide_angle )
	{
		// Furthermore, check if z-value of gripper is too low which makes it vulnerable to getting stuck at ground terrain
		const bool is_start_point_prone = (wide_angle && gripper_interface_->isPoseTooCloseToTerrain(world_to_grasp_frame));
		if ( is_start_point_prone )  // adjust the current gripper position (move up), before moving to the target pose
		{
			gripper_evasion_config_ = gripper_interface_->getCurrentConfig();  // set gripper_evasion_config_ to current config
			gripper_evasion_target_ = world_to_grasp_frame;
			start_evasions_++;  // DEBUGGING
			return true;
		}
		else  // check whether target pose is ok
		{
			const bool is_end_point_prone = (wide_angle && gripper_interface_->isPoseTooCloseToTerrain(target_pose));

			if ( is_end_point_prone )  // adjust the target position (move it up), before moving to the original target pose
			{
				gripper_interface_->computeConfigFromCartesian(target_pose, gripper_evasion_config_);  // set gripper_evasion_config_ to target config
				gripper_evasion_target_ = target_pose;
				end_evasions_++;  // DEBUGGING
				return true;
			}
		}
	}

	return false;
}

void SimulationBackend::triggerEvasionManeuver()
{
	pre_gripper_evasion_state_ = simulation_state_;
	simulation_state_ = ST_GRIPPER_EVASION_MANEUVER;
}

void SimulationBackend::toggleCameraState(const bool &new_state)
{
	camera_sensor_->depth = new_state;
	camera_sensor_->rgb = new_state;
}

bool SimulationBackend::callSaveGraspdataService(const std::string &object_name, const std::size_t &view_counter, const sensor_msgs::Image &depth_image,
							  const std::vector<EvaluatedGrasp> &evaluated_grasps)
{
	// DEBUGGING
	/*const bool apply_neural_network = false;
	if ( apply_neural_network )
	{
		ipa_grasppose_srvs::PredictGraspQualities predict_srv;
		predict_srv.request.depth_image = depth_image;
		for ( const EvaluatedGrasp &grasp_info : evaluated_grasps )
			predict_srv.request.grasp_poses_camera_frame.push_back(grasp_info.grasp_pose->grasp_pose_msg_camera_frame);

		bool response = ros::service::call("/ipa_grasppose_dnn/predict_graspquality_srv", predict_srv);

		std::vector<float> &success_probs = predict_srv.response.success_probabilities;
		for ( size_t i=0; i<success_probs.size(); ++i )
			std::cout << "Success probability: " << success_probs[i] << "/" << evaluated_grasps[i].rating << std::endl;

		std::cout << "DONE EVALUATING: " << response << std::endl;
		return true;
	}*/
	// End debugging

	ipa_grasppose_srvs::SaveGraspData save_srv;
	save_srv.request.object_name = object_name;  // hand over unique name
	save_srv.request.view_counter = view_counter;  // current view to differentiate between them (each view of an object with its grasp poses will be saved once)
	save_srv.request.depth_image = depth_image;
	save_srv.request.grasp_poses_camera_frame.reserve(evaluated_grasps.size());
	save_srv.request.grasp_poses_evaluations.reserve(evaluated_grasps.size());
	for ( const EvaluatedGrasp &grasp_info : evaluated_grasps )
	{
		save_srv.request.grasp_poses_camera_frame.push_back(grasp_info.grasp_pose->grasp_pose_msg_camera_frame);
		save_srv.request.grasp_poses_evaluations.push_back(grasp_info.rating);
	}

	if ( !save_graspposedata_client_.call(save_srv) )
	{
		ROS_WARN("SimulationBackend::callSaveGraspdataService - Failed to call save_graspdata_srv service.");
		failed_save_calls_++;
		return false;
	}

	total_grasp_poses_ += evaluated_grasps.size();  // DEBUGGING

	return true;
}

bool SimulationBackend::setNextAugmentedPerspective()
{
	if ( perspective_augmentation_counter_ >= current_delta_poses_set_->size() )
	{
		ROS_WARN("SimulationBackend::setNextAugmentedPerspective - All perspectives for augmentation have already been visited.");
		return false;
	}

	const Math3D::RigidTransform &delta_pose = (*current_delta_poses_set_)[perspective_augmentation_counter_++];
	camera_sensor_->Tsensor = original_camera_pose_ * delta_pose;
	return true;
}

bool SimulationBackend::isGraspPoseWithinPointCloud(const Math3D::RigidTransform &pose, const vector<Math3D::Vector3> &point_cloud)
{
	const Math3D::Vector3 grasp_direction_camera = pose.R * gripper_interface_->getGraspDirectionVector();  // grasp direction in camera sensor coordinates
	const double threshold_sq = 0.0025*0.0025;  // 0.25cm squared -> 0.25cm radius around line is considered a valid pcl point

	for ( const Math3D::Vector3 &point : point_cloud )
	{
		// Get minimum distance of point to line (line: pose.t + scaling*grasp_direction, scaling = scalar)
		// Scaling is the projection of the pointcloud point onto the line.
		// The grasp pose has been built using the surface normals and that is why the line should connect with one point cloud point unless it is invalid (not seen anymore by camera)
		const double scaling = grasp_direction_camera.dot(point - pose.t);
		const Math3D::Vector3 distance_vector = point - (pose.t + scaling*grasp_direction_camera);

		if ( distance_vector.normSquared() < threshold_sq && gripper_interface_->isInGraspableReach(scaling) )
			return true;
	}

	return false;
}
