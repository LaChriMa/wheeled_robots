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
	
  public:
	double friction;
 	
    ThisSim() {
   	  addPaletteFile("colors/UrbanExtraColors.gpl");
   	  addColorAliasFile("colors/UrbanColorSchema.txt");
   	  addColorAliasFile("colors.txt");
   	  setGroundTexture("Images/whiteground.jpg");
    }


    virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
    {
      setCameraHomePos(Pos(-14,14, 10),  Pos(-135, -24, 0));
	  setCameraMode( Follow );

      global.odeConfig.setParam("simstepsize", 0.01);
      global.odeConfig.setParam("controlinterval", 1);
      global.odeConfig.setParam("gravity", -9.8);
	  global.odeConfig.setParam("noise", 0);
	  global.odeConfig.addParameterDef("friction", &friction, 0.1, "parameter for velocity depending friction");


   	  DifferentialConf conf = Differential::getDefaultConf();
   	  conf.wheelMass 		  = .5;
      conf.wheelRadius 		  = .3;
      conf.wheelHeight        = .1; 
      conf.bodyRadius         = 1.; 
      conf.bodyHeight         = .5; 
      conf.bodyMass           = 1.; 
      conf.sWheelMass          = 0.00001;
      conf.initWheelOrientation = 0; //M_PI/2.; //M_PI/4.0;

	  OdeHandle robHandle = odeHandle;
	  //robHandle.substance.toRubber(5);
   	  auto robot = new Differential(robHandle, osgHandle, conf, "Two wheeled robot");
   	  robot->addSensor(std::make_shared<SpeedSensor>(1), Attachment(-1));
   	  robot->place(Pos(0, 0, 0));

   	  auto controller = new BasicController("Basic Controller", global.odeConfig);
   	  auto wiring = new One2OneWiring(new ColorUniformNoise(.1));

	  /** PlotOption will change the interval for how often 
	  **  will be written into the log-file to one */
   	  //auto agent = new OdeAgent( PlotOption(File) );
   	  auto agent = new OdeAgent(global);

   	  agent->init(controller, robot, wiring);
   	  global.agents.push_back(agent);
   	  global.configs.push_back(agent);
		  

    }

    virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
	  if(!pause)
	  {
		 OdeRobot* rob = globalData.agents[0]->getRobot();
		 Pos vel = rob->getMainPrimitive()->getVel();
		 rob->getMainPrimitive()->applyForce(-vel*friction);
	  }
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
