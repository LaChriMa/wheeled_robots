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
#include <ode_robots/octaplayground.h>
#include <ode_robots/passivebox.h>
#include <ode_robots/randomobstacles.h>

#include <selforg/agent.h>

// Four-wheeled cars
#include <ode_robots/nimm4.h>
#include "nimm4angle.h"
#include "couplingrodneuron.h"
#include "carchain.h"
#include <ode_robots/fourwheeled.h>


#include <string>

using namespace lpzrobots;

class ThisSim : public Simulation
{
	
  public:
	double friction;  /** velocity depending friction factor */
	std::string env = "no";  /** "wall", "playground" or "no" */
	bool randObstacles = false;
 	
    ThisSim() {
   	  addPaletteFile("colors/UrbanExtraColors.gpl");
   	  addColorAliasFile("colors/UrbanColorSchema.txt");
   	  addColorAliasFile("colors.txt");
   	  setGroundTexture("Images/whiteground.jpg");
    }


    virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
    {
	  /**** GLOBAL SETTINGS ****/
      setCameraHomePos(Pos(-14,14, 10),  Pos(-135, -24, 0));
      setCameraHomePos(Pos(-2.24309, 0.920369, 1.45178),  Pos(-113.025, -29.863, 0));
	  setCameraMode( Follow );

      global.odeConfig.setParam("simstepsize", 0.001);
      global.odeConfig.setParam("controlinterval", 1);
      global.odeConfig.setParam("gravity", -9.8);
	  global.odeConfig.setParam("noise", 0);
	  global.odeConfig.addParameterDef("friction", &friction, 0.1, "parameter for velocity depending friction");
	  /**** END GLOBAL SETTINGS ****/


	  /**** WHEELED ROBOT ****/
   	  //DifferentialConf conf = Differential::getDefaultConf();
   	  //conf.wheelMass 		  = .5;
      //conf.wheelRadius 		  = .3;
      //conf.wheelHeight        = .1; 
      //conf.initWheelOrientation = 0.; //M_PI/2.; //M_PI/4.0;
      //conf.bodyRadius         = 1.; 
      //conf.bodyHeight         = .5; 
      //conf.bodyMass           = 1.; 
      //conf.supportWheels      = true; 
	  //conf.sphericalSupportWheels 		= true;
      //conf.sWheelMass         = 0.00001;
	  //OdeHandle robHandle = odeHandle;
	  ////robHandle.substance.toRubber(5);
   	  //auto robot = new Differential(robHandle, osgHandle, conf, "Two wheeled robot");
   	  //robot->addSensor(std::make_shared<SpeedSensor>(1), Attachment(-1));
   	  //robot->place(Pos(0, 0, 0));
   	  //auto controller = new BasicController("Basic Controller", global.odeConfig);
   	  //auto wiring = new One2OneWiring(new ColorUniformNoise(.1));
   	  ////auto agent = new OdeAgent( PlotOption(File) );  /** set the frequency for writing in the log-file*/
   	  //auto agent = new OdeAgent(global);
   	  //agent->init(controller, robot, wiring);
   	  //global.agents.push_back(agent);
   	  //global.configs.push_back(agent);
	  /**** END WHEELED ROBOT ****/
	  
      /*** CAR CHAIN ****/
      CarChainConf conf = CarChain::getDefaultConf();
      conf.carNumber     = 5;
      auto robot = new CarChain( odeHandle, osgHandle, conf, "Train");
   	  robot->place(Pos(0, 0, 0));
   	  auto controller = new CouplingRod("Coupling_Rod", global.odeConfig);
   	  auto wiring = new One2OneWiring(new ColorUniformNoise(.1));
   	  auto agent = new OdeAgent(global);
   	  //auto agent = new OdeAgent( PlotOption(File) );
   	  agent->init(controller, robot, wiring);
   	  global.agents.push_back(agent);
   	  global.configs.push_back(agent);
      TrackRobot* TrackOpt = new TrackRobot(false,false,false,true);
      TrackOpt->conf.displayTraceDur = 1000;
      TrackOpt->conf.displayTraceThickness = 0.;
      agent->setTrackOptions( *TrackOpt );
	  /*** END CAR CHAIN ****/


      /**** FOUR WHEELED ROBOTS ****/
      //auto N4robot = new Nimm4Angle( odeHandle, osgHandle, "Nimm4Angle");
      //N4robot->place( Pos(0,2,0) );
      //auto N4controller = new Cont4Wheels("Nimm4Controller", global.odeConfig);
      //N4controller->init(4,4);
      //auto N4wiring = new One2OneWiring( new ColorUniformNoise(.1));
      //auto N4agent = new OdeAgent(global);
      //N4agent->init( N4controller, N4robot, N4wiring );
      //global.agents.push_back(N4agent);
      //global.configs.push_back(N4agent);
      /**** END FOUR WHEELED ROBOTS ****/

      /**** FOUR WHEELED ROBOTS ****/
      //FourWheeledConf FWconf = FourWheeled::getDefaultConf();
      //auto Zrobot = new FourWheeled( odeHandle, osgHandle, FWconf, "FourWheeled");
      //Zrobot->place(Pos(2,0,0));
      //// controller noch nicht angepasst
      //auto Zcontroller = new BasicController("Z:4 Wheeled Controller", global.odeConfig);
      //auto Zwiring = new One2OneWiring( new ColorUniformNoise(.1));
      //auto Zagent = new OdeAgent(global);
      //Zagent->init(Zcontroller, Zrobot, Zwiring);
      //global.agents.push_back(Zagent);
      //global.configs.push_back(Zagent);
      /**** END FOUR WHEELED ROBOTS ****/


	  /*** ENVIRONMENT ***/
	  if( env == "wall" )
	  { 
	    auto* box = new Box(15,0.3,1.5);
	    box->init( odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
	    //box->setSubstance(  Substance::getPlastic(0.8) );   //  Substance::getRubber(25) ); 
	    box->setSubstance(  Substance::getRubber(25) ); 
	    box->setPosition( Pos(0,30,0) );
        setCameraHomePos(Pos(-0.0465025, -23.5645, 9.7256),  Pos(1.90295, -20.8458, 0));
	  }
	  if( env == "playground" )
	  { 
		OdeHandle wallHandle = odeHandle;
		//wallHandle.substance.toPlastic(0.8);
		wallHandle.substance.toRubber(25);
        OctaPlayground* world = new OctaPlayground( wallHandle, osgHandle, 
													Pos(20,0.2,0.5), 15, false);
        world->setPose( osg::Matrix::translate(0,0,0) );
        global.obstacles.push_back( world );
        setCameraHomePos(Pos(0.149249, 0.434834, 30.4279),  Pos(179.487, -89.6461, 0));
        setCameraHomePos(Pos(6.80232, 46.9342, 34.5286),  Pos(173.018, -40.2009, 0));
        setCameraMode( Static );
	
		RandomObstaclesConf randConf = RandomObstacles::getDefaultConf();
    	randConf.pose = osg::Matrix::translate(0,0,0);
    	randConf.area = Pos(15,15,2);
    	randConf.minSize = Pos(4.,2.,1.);
    	randConf.maxSize = Pos(4.,5.,1.);
    	randConf.minDensity = 10;
    	randConf.maxDensity = 10;
    	RandomObstacles* RandObstacle = new RandomObstacles(wallHandle, osgHandle, randConf);
    	/** Generation an placing Objects */
    	if(randObstacles == true){
          int num_randObs = 4;
          for (int i=0; i< num_randObs; i++){
             RandObstacle->spawn(RandomObstacles::Box, RandomObstacles::Foam);
             global.obstacles.push_back( RandObstacle );
          }
    	}
	  }
	  /*** End ENVIRONMENT ***/
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
	  if (down) 
	  {
		OdeRobot* rob = globalData.agents[0]->getRobot();
		switch( (char) key)
	 	{ 
		  case 'j': dBodyAddForce( rob->getMainPrimitive()->getBody(), 200, 0, 0); break;
		  case 'J': dBodyAddForce( rob->getMainPrimitive()->getBody(), -200, 0, 0); break;
		  case 'k': dBodyAddForce( rob->getMainPrimitive()->getBody(), 0, 200, 0); break;
		  case 'K': dBodyAddForce( rob->getMainPrimitive()->getBody(), 0, -200, 0); break;
		  case 'l': dBodyAddTorque( rob->getMainPrimitive()->getBody(), 0, 0, 250); break;
		  case 'L': dBodyAddTorque( rob->getMainPrimitive()->getBody(), 0, 0, -250); break;
	 	}
	  }
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
