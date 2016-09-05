/***************************************************************************
 *   Copyright (C) 2005-2013 LpzRobots development team                    *
 *   Authors:                                                              *
 *    Simón Smith <artificialsimon at ed dot ac dot uk>                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 ***************************************************************************/

#include <ode_robots/simulation.h>
#include <selforg/noisegenerator.h>
#include <ode_robots/odeagent.h>
#include "differential.h"
#include "barrel.h"
#include <ode_robots/speedsensor.h>

#include "basiccontroller.h"
#include <selforg/one2onewiring.h>

#include <ode_robots/playground.h>
#include <ode_robots/passivebox.h>

#include <selforg/agent.h>

#include <string>

using namespace lpzrobots;

class ThisSim : public Simulation
{
  std::string robot = "barrel";
	
  public:
    ThisSim() {
      // set Title of simulation
      setTitle("test version");
   	  //addPaletteFile("colors/UrbanExtraColors.gpl");
   	  //addColorAliasFile("colors/UrbanColorSchema.txt");
   	  //addColorAliasFile("colors.txt");
   	  //setGroundTexture("Images/whiteground.jpg");
    }
    ~ThisSim() { }

    /// start() is called at the start and should create all the object (obstacles, agents...).
    virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
    {
      // Initial position and orientation of the camera (use 'p' in graphical window to find out)
      setCameraHomePos(Pos(-14,14, 10),  Pos(-135, -24, 0));
	  setCameraMode( Follow );
      // Some simulation parameters can be set here
      global.odeConfig.setParam("simstepsize", 0.01);
      global.odeConfig.setParam("controlinterval", 1);
      global.odeConfig.setParam("gravity", -9.8);
	  global.odeConfig.setParam("noise", 0);

	  if( robot=="differential" ) {
	  	// Get the default configuration of the robot
   	  	DifferentialConf conf = Differential::getDefaultConf();
   	  	conf.wheelMass = .5;
      	conf.wheelRadius = .3;
	  	//conf.wheelMotorPower = 1;
	  	//conf.wheelMotorMaxSpeed = 1;
   	  	// Instantiating the robot
	  	OdeHandle robHandle = odeHandle;
	  	//robHandle.substance.toRubber(5);
   	  	auto robot = new Differential(robHandle, osgHandle, conf, "Differential robot");
   	  	// add a speed sensor to the robot (attached to the "main primitive" (-1)
   	  	// (specifiy index if needed)
   	  	robot->addSensor(std::make_shared<SpeedSensor>(1), Attachment(-1));
   	  	// Placing the robot in the scene
   	  	//robot->place( osg::Matrix::rotate(1,1,1,1)*osg::Matrix::translate(1,2,1) );
   	  	robot->place(Pos(0, 0, 0));

   	  	// Instantiating the controller
   	  	auto controller = new BasicController("Basic Controller", global.odeConfig);
   	  	// Create the wiring with color noise
   	  	auto wiring = new One2OneWiring(new ColorUniformNoise(.1));

   	  	// Create Agent
   	  	//auto agent = new OdeAgent(global);
	  	/** PlotOption will change the interval for how often will be written into the log-file to one */
   	  	auto agent = new OdeAgent( PlotOption(File) );

   	  	// Agent initialisation
   	  	agent->init(controller, robot, wiring);
   	  	// Adding the agent to the agents list
   	  	global.agents.push_back(agent);
   	  	global.configs.push_back(agent);
	  }
	  else if( robot=="barrel" ) {
   	  	BarrelConf conf = Barrel::getDefaultConf();
   	  	conf.bodyMass = .5;
      	conf.bodyRadius = .3;
      	conf.bodyHeight = .6;
      	conf.rollingFriction = 0.1;
      	conf.torque= 1.0;
	  	OdeHandle robHandle = odeHandle;
   	  	auto robot = new Barrel(robHandle, osgHandle, conf, "Barrel robot");
   	  	robot->addSensor(std::make_shared<SpeedSensor>(1), Attachment(-1));
		robot->addSensor(std::make_shared<SpeedSensor>( 1, SpeedSensor::Rotational ), Attachment(-1));
   	  	//robot->place( osg::Matrix::rotate(M_PI,0,0,0)*osg::Matrix::translate(1,2,1) );
   	  	robot->place(Pos(0, 0, 0));

   	  	// Instantiating the controller
   	  	auto controller = new BasicController("Basic Controller", global.odeConfig);
   	  	// Create the wiring with color noise
   	  	auto wiring = new One2OneWiring(new ColorUniformNoise(.1));

   	  	// Create Agent
   	  	auto agent = new OdeAgent(global);
	  	/** PlotOption will change the interval for how often will be written into the log-file to one */
   	  	//auto agent = new OdeAgent( PlotOption(File) );

   	  	// Agent initialisation
   	  	agent->init(controller, robot, wiring);
   	  	// Adding the agent to the agents list
   	  	global.agents.push_back(agent);
   	  	global.configs.push_back(agent);
	
	  }

	  

      //// New playground
      //auto playground = new Playground(odeHandle, osgHandle,osg::Vec3(15., .2, 1.2), 1);
      //// Set colours
      //playground->setGroundColor(Color(.784, .784, .0));
      //playground->setColor(Color(1., .784, .082, .3));
      //// Set position
      //playground->setPosition(osg::Vec3(.0, .0, .1));
      //// Adding playground to obstacles list
      //global.obstacles.push_back(playground);

    }

    /* Functions not used in this tutorial but typically useful */
    virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    }

    virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down) {
      return false;
    }

    virtual void bindingDescription(osg::ApplicationUsage & au) const {
    }
};

int main (int argc, char **argv)
{
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;
}
