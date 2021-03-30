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

// Does the update step of the simulation

#ifndef GRASP_VIEW_PROGRAM_H
#define GRASP_VIEW_PROGRAM_H


#include <Klampt_grasp_simulator/view_program_base.h>
#include <Klampt_grasp_simulator/simulation_backend.h>


class GraspViewProgram : public ViewProgramBase
{
private:

	shared_ptr<Robot> camera_;
	CameraSensor* camera_sensor_;


public:

	SimulationBackend &simulation_backend_;
	bool simulate_;
	double dt_;

	GraspViewProgram(SimulationBackend &backend);
	virtual ~GraspViewProgram();

	// Overwritten methods
	virtual bool Initialize();
	virtual void Handle_Idle();
	virtual void RenderWorld();  // adds drawing of camera frame
	virtual void Handle_Keypress(unsigned char key,int x,int y);

	virtual int runWithoutGUI();
	bool loadAndInitSim();

};

#endif // GRASP_VIEW_PROGRAM_H
