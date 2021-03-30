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


#include <Klampt_grasp_simulator/view_program_base.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <GL/glui.h>


ViewProgramBase::ViewProgramBase(RobotWorld* world, WorldSimulation* simulation) :
					world_(world), simulation_(simulation)
{
	// Change resolution of simulation window
	//width = 1280;
	//height = 720;
}

bool ViewProgramBase::Initialize()
{
	camera.dist = 3.0;
	viewport.n = 0.1;
	viewport.f = 100;
	viewport.setLensAngle(DtoR(60.0));

	// Add some ambient brightness to avoid total darkness -> bad for HSV color filter (dark object segments could be filtered out)
	GLfloat ambient_brightness[] = {0.35f, 0.35f, 0.35f, 1.0f};
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient_brightness);

	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glClearColor(world_->background.rgba[0], world_->background.rgba[1], world_->background.rgba[2], world_->background.rgba[3]);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	GLUINavigationProgram::Initialize();
}

void ViewProgramBase::RenderWorld()  // copied from SimViewProgram.h
{
	glDisable(GL_LIGHTING);
	GLDraw::drawCoords(0.1);
	glEnable(GL_LIGHTING);
	for(size_t i=0; i<world_->terrains.size(); i++)
		world_->terrains[i]->DrawGL();
	for(size_t i=0; i<world_->rigidObjects.size(); i++)
		world_->rigidObjects[i]->DrawGL();

	for(size_t i=0; i<world_->robots.size(); i++)
	{
		world_->robotViews[i].PushAppearance();
		for(size_t j=0; j<world_->robots[i]->links.size(); j++)
		{
			simulation_->odesim.robot(i)->GetLinkTransform(j, world_->robots[i]->links[j].T_World);
			float color[4] = {0.5,0.5,0.5,1.0};
			if( i==0 )
			{
				Real kg=simulation_->ContactForce(world_->RobotLinkID(i,j)).norm()/9.8;
				Assert(!(kg < 0.0));
				kg /= world_->robots[i]->GetTotalMass();
				Real green = 0.1, yellow = 1.0, red = 1.5;
				if(kg < green)  // grey->green
				{
					color[0]=0.5-0.5*kg/green;
					color[1]=0.5+0.5*kg/green;
					color[2]=0.5-0.5*kg/green;
				}
				else if(kg < yellow)  // green->yellow
				{
					Real u=(kg-green)/(yellow-green);
					color[0]=u;
					color[1]=1.0;
					color[2]=0;
				}
				else if(kg < red)  // yellow->red
				{
					Real u=(kg-yellow)/(red-yellow);
					color[0]=u;
					color[1]=1.0-u;
					color[2]=0;
				}
				else {
					color[0]=1.0;
					color[1]=0;
					color[2]=0;
				}
			}
			world_->robotViews[i].SetColor(j, GLDraw::GLColor(color));
			world_->robotViews[i].DrawLink_World(j);
		}
		world_->robotViews[i].PopAppearance();
	}
}

void ViewProgramBase::DoFreeDrag(int dx, int dy, int button)
{
	if( button == GLUT_LEFT_BUTTON )
		GLUINavigationProgram::DragRotate(dx,dy);
}

void ViewProgramBase::DoCtrlDrag(int dx, int dy, int button)
{
	if( button == GLUT_LEFT_BUTTON )
		GLUINavigationProgram::DragPan(dx,dy);
}

void ViewProgramBase::DoAltDrag(int dx, int dy, int button)
{
	if( button == GLUT_LEFT_BUTTON )
		GLUINavigationProgram::DragZoom(dx,dy);
}

void ViewProgramBase::DoShiftDrag(int dx, int dy, int button)
{
	if( button == GLUT_LEFT_BUTTON )
		camera.dist *= (1 + 0.01*Real(dy));
}

void ViewProgramBase::SetWorldLights()
{
	world_->SetGLLights();
}

void ViewProgramBase::ClickRay(int x, int y, Ray3D& r) const
{
	viewport.getClickSource(x,viewport.y+viewport.h-y,r.source);
	viewport.getClickVector(x,viewport.y+viewport.h-y,r.direction);
}


Robot* ViewProgramBase::ClickRobot(const Ray3D& r, int& body, Vector3& localpt) const
{
	return world_->RayCastRobot(r,body,localpt);
}

RigidObject* ViewProgramBase::ClickObject(const Ray3D& r, Vector3& localpt) const
{
	return world_->RayCastObject(r,localpt);
}

void ViewProgramBase::RefreshIdle()
{
	SleepIdleCallback(0);
}

