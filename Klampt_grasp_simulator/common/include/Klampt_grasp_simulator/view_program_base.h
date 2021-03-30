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

// Offers basic functionality to maneuver through the simulation and render the scene

#ifndef VIEW_PROGRAM_BASE_H
#define VIEW_PROGRAM_BASE_H


#include <KrisLibrary/GLdraw/GLUINavigationProgram.h>
#include <KrisLibrary/math3d/Ray3D.h>
#include <Klampt/Modeling/World.h>
#include <Klampt/Simulation/WorldSimulation.h>


class ViewProgramBase : public GLUINavigationProgram
{
public:

	RobotWorld* world_;
	WorldSimulation* simulation_;

	ViewProgramBase(RobotWorld* world, WorldSimulation* simulation);

	virtual bool Initialize();
	virtual void RenderWorld();  // Renders the state of the simulation
	virtual void DoFreeDrag(int dx, int dy, int button);
	virtual void DoCtrlDrag(int dx, int dy, int button);
	virtual void DoAltDrag(int dx, int dy, int button);
	virtual void DoShiftDrag(int dx, int dy, int button);
	virtual void SetWorldLights();
	void ClickRay(int x, int y, Math3D::Ray3D& r) const;
	Robot* ClickRobot(const Math3D::Ray3D& r, int& body, Math3D::Vector3& localpt) const;
	RigidObject* ClickObject(const Math3D::Ray3D& r, Math3D::Vector3& localpt) const;
	virtual void RefreshIdle();

};

#endif // VIEW_PROGRAM_BASE_H
