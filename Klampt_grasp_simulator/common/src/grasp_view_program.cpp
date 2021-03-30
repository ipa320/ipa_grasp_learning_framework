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


#include <Klampt_grasp_simulator/grasp_view_program.h>
#include <KrisLibrary/utils.h>
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/robotics/RobotKinematics3D.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/camera/camera.h>
#include <KrisLibrary/math3d/AABB3D.h>
#include <Klampt/View/ViewCamera.h>
#include <GL/glui.h>
#include <iostream>


GraspViewProgram::GraspViewProgram(SimulationBackend &backend) :
				ViewProgramBase(backend.getWorld(), backend.getSimulation()), camera_(nullptr), camera_sensor_(nullptr), simulation_backend_(backend),
				simulate_(false), dt_(backend.getDeltaTime())
{
}

GraspViewProgram::~GraspViewProgram()
{
}

bool GraspViewProgram::Initialize()
{
	ViewProgramBase::Initialize();  // call to parent to properly initialize openGL
	simulate_ = true;
	return true;
}

void GraspViewProgram::Handle_Keypress(unsigned char key,int x,int y)
{
	switch ( key )
	{
		case 'p':  // Pause function
			simulate_ = !simulate_;
			simulate_ ? std::cout << "SIMULATION UNPAUSED" << std::endl : std::cout << "SIMULATION PAUSED" << std::endl;
			break;
		default:
			GLUINavigationProgram::Handle_Keypress(key, x, y);
	}
}

void GraspViewProgram::RenderWorld()
{
	ViewProgramBase::RenderWorld();

	if ( !simulation_backend_.isInitialized() )
		return;

	if ( camera_ != nullptr && camera_sensor_ != nullptr && simulation_backend_.drawCamera() )  // draw camera, frustrum and its pose
	{
		// Draw camera icon and frustrum
		Camera::Viewport vp;
		camera_sensor_->GetViewport(vp);
		vp.xform = camera_->links[camera_sensor_->link].T_World * vp.xform;
		ViewCamera view;
		view.drawFrustum = simulation_backend_.drawCameraFrustum();
		view.DrawGL(vp);

		// Draw camera frame
		Math3D::RigidTransform camera_frame = camera_->links[camera_sensor_->link].T_World * camera_sensor_->Tsensor;
		glDisable(GL_LIGHTING);
		glPushMatrix();
		GLDraw::glMultMatrix((Matrix4)camera_frame);
		GLDraw::drawCoords(0.1);
		glPopMatrix();
	}

	// Draw gripper grasp frame
	glDisable(GL_LIGHTING);  // avoid that frames and bounding box drawing is affected by lighting
	if ( simulation_backend_.drawGripperGraspFrame() )
	{
		Math3D::Matrix4 frame = simulation_backend_.getWorldToGripperGraspFrame();
		if ( !frame.isZero() )
		{
			glPushMatrix();
			GLDraw::glMultMatrix(frame);
			GLDraw::drawCoords(0.1);
			glPopMatrix();
		}
	}

	// Draw current grasp pose
	if ( simulation_backend_.drawCurrentGraspPose() )
	{
		Math3D::Matrix4 pose = simulation_backend_.getWorldToCurrentGraspPose();
		if ( !pose.isZero() )
		{
			glPushMatrix();
			GLDraw::glMultMatrix(pose);
			GLDraw::drawCoords(0.1);
			glPopMatrix();
		}
	}
	else if ( simulation_backend_.drawAllGraspPoses() )  // Draw all grasp poses
	{
		std::vector<Math3D::Matrix4> grasp_poses;
		simulation_backend_.getWorldToGraspPoses(grasp_poses);
		for ( const Math3D::Matrix4 &pose : grasp_poses )
		{
			glPushMatrix();
			GLDraw::glMultMatrix(pose);
			GLDraw::drawCoords(0.01);
			glPopMatrix();
		}
	}

	// Draw bounding box of current object
	if ( simulation_backend_.drawBoundingBox() )
	{
		Math3D::AABB3D current_aabb;
		if ( simulation_backend_.getCurrentBoundigBox(current_aabb) )
		{
			glColor3f(1, 0, 0);
			GLDraw::drawWireBoundingBox(current_aabb.bmin, current_aabb.bmax);
		}
	}

	// DEBUGGING
	{
		const bool draw_pregrasp_pose = false;
		if ( draw_pregrasp_pose )
		{
			Math3D::Matrix4 pose = simulation_backend_.getWorldToCurrentPreGraspPose();
			if ( !pose.isZero() )
			{
				glPushMatrix();
				GLDraw::glMultMatrix(pose);
				GLDraw::drawCoords(0.1);
				glPopMatrix();
			}
		}

		const bool center_camera = false;
		if ( center_camera )
		{
			Math3D::Vector3 aabb_center = simulation_backend_.getOriginalAABBCenter();
			if ( !aabb_center.isZero() )
			{
				Camera::Viewport vp;
				camera_sensor_->GetViewport(vp);
				vp.xform = camera_->links[camera_sensor_->link].T_World * vp.xform;
				camera.fromCamera(vp, 1.0);
				camera.tgt = aabb_center;
			}
		}

		const bool draw_camera_poses = false;
		if ( draw_camera_poses )
		{
			std::vector<Math3D::Matrix4> camera_poses;
			simulation_backend_.getAllCameraPoses(camera_poses);
			if ( !camera_poses.empty() )
			{
				for ( const Math3D::Matrix4 &pose : camera_poses )
				{
					glPushMatrix();
					GLDraw::glMultMatrix(pose);
					GLDraw::drawCoords(0.1);
					glPopMatrix();
				}
			}
		}
	}
	// End debugging
}

void GraspViewProgram::Handle_Idle()
{
	if( simulation_backend_.isInitialized() && simulate_ )
	{
		Timer timer;

		simulation_backend_.advanceSimulationState();
		simulation_->Advance(dt_);
		Refresh();

		SleepIdleCallback(int(Max(0.0, dt_-timer.ElapsedTime())*1000.0));
	}
	GLUINavigationProgram::Handle_Idle();
}

bool GraspViewProgram::loadAndInitSim()
{
	if ( !simulation_backend_.initBackend() )
		return false;

	camera_ = simulation_backend_.getCamera();
	camera_sensor_ = simulation_backend_.getCameraSensor();
}

int GraspViewProgram::runWithoutGUI()
{
	if ( !simulation_backend_.isInitialized() )
		return -1;

	return simulation_backend_.runSimulation();
}
