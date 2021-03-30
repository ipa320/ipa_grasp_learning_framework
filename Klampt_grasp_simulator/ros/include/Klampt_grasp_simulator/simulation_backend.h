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

#ifndef SIMULATOR_ENVIRONMENT_H
#define SIMULATOR_ENVIRONMENT_H


#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math3d/rotation.h>
#include <KrisLibrary/robotics/RobotKinematics3D.h>
#include <Klampt/Modeling/World.h>
#include <Klampt/Modeling/RigidObject.h>
#include <Klampt/Simulation/ODERigidObject.h>
#include <Klampt/Simulation/WorldSimulation.h>
#include <Klampt/Interface/SimulationGUI.h>
#include <Klampt/Sensing/VisualSensors.h>
#include <Klampt/Planning/PlannerSettings.h>
#include <Klampt_grasp_simulator/calculation_utilities.h>
#include <Klampt_grasp_simulator/gripper_interface_base.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <vector>

#include <chrono>  // DEBUGGING


// States for state machine in simulation loop
enum SimulationState
{
	ST_OBJECT_SPAWN_NEXT		= 0,
	ST_OBJECT_WAIT				= 1,
	ST_CAMERA_SET_POSE			= 2,
	ST_GRIPPER_MOVE_TO_START	= 3,
	ST_GRIPPER_EVASION_MANEUVER	= 4,
	ST_GET_GRIPPERPOSES			= 5,
	ST_GRIPPER_PREPARE_GRASP	= 6,
	ST_GRIPPER_MOVE_TO_GRASP	= 7,
	ST_GRIPPER_CLOSING			= 8,
	ST_GRIPPER_LIFT_OBJECT		= 9,
	ST_ERROR_STATE				= 10,
	ST_ASSESS_GRASP				= 11,
	ST_RESET_SIM				= 12,
	ST_PERSPECTIVE_AUGMENTATION = 13,
	ST_FINISHED					= 14
};

// Struct that contains a pose of a rigid object and the object's name
struct RigidObjectDefaults
{
	std::string file_name;
	std::string object_name;  // built from file_name and name_tag
	size_t camera_flight_index;  // assigned camera flight to each object
	Math3D::Vector3 position;
	Math3D::EulerAngleRotation rotation;

	RigidObjectDefaults(const std::string &file, const std::string &object, const size_t &flight_idx, const double &x, const double &y, const double &z,
						const double &yaw, const double &pitch, const double &roll) :
						file_name(file), object_name(object), camera_flight_index(flight_idx), position(x, y, z), rotation(yaw, pitch, roll)
	{
	}
};

// Struct that contains a pair of pre grasp pose (gripper start pose) and the final grasp pose
// Furthermore the original grasp pose of the sampler will be stored so it does not have to be recalculated when saving a grasp pose to drive.
struct GripperGraspPose
{
	Math3D::RigidTransform pre_grasp_pose;
	Math3D::RigidTransform grasp_pose;
	geometry_msgs::Pose grasp_pose_msg_camera_frame;
};

// Configuration with a time-stamp, used for stuck detection
struct StampedConfig
{
	Config configuration;
	double time_stamp;
};

// Struct that contains a pointer to a grasp pose (to one within gripper_grasp_poses_) and its corresponding rating (good/bad)
struct EvaluatedGrasp
{
	GripperGraspPose* grasp_pose;
	bool rating;
};

// Set of delta poses which are used for perspective augmentation
// Each set is assigned to a unique radius
struct DeltaPosesSet
{
	double radius;  // distance of camera to object -> radius in spherical coordinates
	std::vector<Math3D::RigidTransform> delta_poses;  // local positions in camera frame and global rotations
};

class SimulationBackend
{
protected:

	bool initialized_;
	ros::NodeHandle *node_handle_;
	RobotWorld *world_;
	SimGUIBackend gui_backend_;
	WorldSimulation* simulation_;
	WorldPlannerSettings* collision_detection_;
	GripperInterfaceBase* gripper_interface_;
	int camera_index_;
	shared_ptr<Robot> camera_;
	CameraSensor* camera_sensor_;
	std::vector<RigidObjectDefaults> rigid_object_database_;	// List containing all rigid objects that will be tested for graspability
	RigidObjectDefaults* current_rigid_object_defaults_;
	SimulationState simulation_state_;
	SimulationState pre_gripper_evasion_state_;  // used to recover to original state after visiting state ST_GRIPPER_EVASION_MANEUVER
	size_t camera_flight_counter_;
	std::vector<std::vector<Math3D::RigidTransform>> active_camera_flights_;  // Flight poses of the camera: Holds positions offsets of the sphere (in world coordinates) and rotations (in camera coordinates)
	std::vector<Math3D::RigidTransform>* current_camera_flight_;  // Pointer to the current camera flight
	std::vector<DeltaPosesSet> perspective_augmentation_;
	std::vector<Math3D::RigidTransform>* current_delta_poses_set_;  // Pointer to the current delta poses set
	size_t perspective_augmentation_counter_;  // counting through current set of perspectives for augmentation
	size_t saved_perspective_augmentations_;  // counting all additionally saved perspectives for augmentation, used to create a unique view counter together with camera_flight_counter_
	bool perspective_augmentation_prepared_;  // used in the ST_PERSPECTIVE_AUGMENTATION state to turn on the camera
	bool recover_to_perspective_augmentation_state_;  // sed to recover from ST_GRIPPER_MOVE_TO_START to ST_PERSPECTIVE_AUGMENTATION state
	Math3D::RigidTransform original_camera_pose_;  // original camera pose (in world frame) which the delta poses will be applied upon to generate new perspectives
	bool gripper_moves_;
	std::vector<GripperGraspPose> gripper_grasp_poses_;
	size_t gripper_poses_counter_;
	Config current_gripper_target_;
	Config gripper_start_configuration_;  // configuration the gripper spawns with, gripper retreats to this configuration before camera captures data
	Config gripper_evasion_config_;  // target configuration of gripper evasion maneuver
	Math3D::RigidTransform gripper_evasion_target_;  // target pose of gripper evasion maneuver
	Math3D::RigidTransform gripper_start_pose_;  // world to gripper transform after spawning gripper, used for evasion maneuver
	bool skip_evasion_once_;  // set when stuck in evasion to avoid getting in an evasion stuck loop (evasion maneuver called in a loop but gripper can't perform maneuver)
	sensor_msgs::Image current_depth_image_;
	std::vector<EvaluatedGrasp> evaluated_grasps_;  // contains all valid grasp poses and their ratings
	bool simulation_has_finished_;

	double dt_;  // step time of simulation
	std::string world_path_;
	std::string rigid_objects_path_;
	size_t rigid_object_database_counter_;
	ODERigidObject* current_rigid_object_;
	CalcUtils::PointcloudConversionInfo pcl_conversion_;
	GripperGraspPose* current_grasp_pose_;
	bool error_flag_;  // Set when something bad happens while grasping
	Math3D::RigidTransform gripper_at_object_pose_;  // pose of opened gripper at object when it reached the grasp pose
	Math3D::RigidTransform original_object_pose_;  // object's pose after object has come to rest after spawning
	Math3D::Vector3 original_object_aabb_center_;  // center of current object's aabb, used for camera centering and evasion maneuver
	Math3D::Vector3 object_to_grasp_frame_;  // object to grasp frame in object frame coordinates, used to check whether object has slipped through gripper
	double last_stuck_check_time_;
	StampedConfig last_stamped_gripper_config_;
	double last_object_movement_time_;
	double grasp_assessment_time_;  // time when simulation enters assessment state
	double movement_timeout_time_;
	unsigned short stuck_counter_;
	Math3D::Vector3 plane_normal_filter_world_;
	bool ignore_last_grasp_;  // Do not evaluate grasps that could not perform well due to collision or a stuck gripper
	double normal_search_radius_;
	float downsampling_voxel_leaf_size_;
	std::string storage_folder_path_;
	std::string simulation_storage_file_;
	std::string simulation_storage_file_path_;

	bool draw_gripper_graspframe_;
	bool draw_all_graspposes_;
	bool draw_current_grasppose_;
	bool draw_bounding_box_;
	bool draw_camera_;
	bool draw_camera_frustum_;

	//Debugging
	std::chrono::high_resolution_clock::time_point t_start_;
	int prepare_errors_;
	int evasion_errors_;
	int move_start_errors_;
	int all_grasps_;
	int valid_grasps_;
	int successful_grasps_;
	int start_evasions_;
	int end_evasions_;
	size_t total_grasp_poses_;
	int failed_save_calls_;
	// End debugging

	ros::ServiceClient sample_graspposes_client_;
	ros::ServiceClient save_graspposedata_client_;
	ros::Publisher camera_info_publisher_;
	sensor_msgs::CameraInfo camera_info_;  // camera info topic of simulated camera


	bool setupRigidObjectDatabase();
	short spawnNextGraspableObject();
	bool setNextCameraPose();
	void getSegmentedPointCloud2AndDepthImage(sensor_msgs::PointCloud2 &segmented_point_cloud2, sensor_msgs::Image &segmented_depth_image);  // ros format pointcloud
	void getSegmentedPointCloudAndDepthImage(std::vector<Math3D::Vector3> &segmented_point_cloud, sensor_msgs::Image &segmented_depth_image);  // Klampt format pointcloud
	bool setInitialTransformOfRigidObject(const RigidObject &object, const RigidObjectDefaults &default_values);
	void setLiftObject();
	bool isObjectLifted();
	bool assessGrasp();  // cehck if current grasp can lift the object
	void storeGrasp(const bool &good_grasp);  // store the current grasp to evaluated_grasps_ with its label
	void disposeCurrentRigidObject();
	void resetCurrentRigidObject();
	bool isCurrentObjectMoving(const double &angular_theshold = 0.001, const double &linear_threshold = 0.001);
	bool hasObjectMovedTooMuch(const double &allowed_position_deviation, const double &allowed_rotation_deviation_angle);
	bool isGripperStuck();
	bool hasGripperTimedOut();
	bool isGripperCollidingWithEnvironment();
	bool hasGripperLostObject(const double &allowed_position_deviation);
	void prepareGripperMovement();
	void triggerErrorState(const bool &ignore_grasp, const std::string &error_msg = "");
	bool isInRelevantHemisphere(const Math3D::Vector3 &candidate, const Math3D::Vector3 &hemisphere_direction);
	bool saveSimulationDataToDrive();
	bool loadSimulationDataFromDrive();
	bool needsEvasionManeuver(const Math3D::RigidTransform &target_pose);  // in some scenarios the gripper can get stuck with the ground floor when changing sides, avoid that using an evasion maneuver
	void triggerEvasionManeuver();
	void toggleCameraState(const bool &new_state);  // turn camera on/off -> improve simulation performance

	// camera_to_world_rotation_at_theta_phi_zero defines the rotation from camera to world at theta=phi=0 (spherical coordinates) -> camera is at [0,0,radius] above object and optical line is pointing downwards.
	void checkAddToDeltaPosesSet(const double &radius, const std::vector<double> &delta_angles, const double &reference_phi, const double &reference_theta,
								 const Math3D::Matrix3 &sensor_to_camera_at_reference_angles, const Math3D::Matrix3 &camera_to_world);
	bool callSaveGraspdataService(const std::string &object_name, const std::size_t &view_counter, const sensor_msgs::Image &depth_image,
								  const std::vector<EvaluatedGrasp> &evaluated_grasps);
	bool setNextAugmentedPerspective();
	bool isGraspPoseWithinPointCloud(const Math3D::RigidTransform &pose, const vector<Math3D::Vector3> &point_cloud);

public:

	SimulationBackend();  // std constructor
	SimulationBackend(ros::NodeHandle* nh, RobotWorld* world);
	~SimulationBackend();
	bool initBackend();
	bool isInitialized();
	int runSimulation();  // run the simulation, used when no gui is set up
	void advanceSimulationState();
	RobotWorld* getWorld();
	WorldSimulation* getSimulation();
	double getDeltaTime();
	shared_ptr<Robot> getCamera();  // used by GraspViewProgram in order to draw last camera frame
	CameraSensor* getCameraSensor();
	Math3D::Matrix4 getWorldToGripperGraspFrame();
	Math3D::Matrix4 getWorldToCurrentGraspPose();

	// DEBUGGING
	Math3D::Matrix4 getWorldToCurrentPreGraspPose();
	Math3D::Vector3 getOriginalAABBCenter();
	void getAllCameraPoses(std::vector<Math3D::Matrix4> &result);
	// End Debugging

	bool getCurrentBoundigBox(Math3D::AABB3D &bounding_box);  // Use AABB (larger estimation) over OBB to avoid pixel dropouts in the border regions of the BB
	void getWorldToGraspPoses(std::vector<Math3D::Matrix4> &result);
	bool simulationHasFinished();
	bool drawGripperGraspFrame();
	bool drawAllGraspPoses();
	bool drawCurrentGraspPose();
	bool drawBoundingBox();
	bool drawCamera();
	bool drawCameraFrustum();

};

#endif // SIMULATOR_ENVIRONMENT_H
