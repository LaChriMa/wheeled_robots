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

// Environment:
#include <ode_robots/playground.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/passivebox.h>
#include <ode_robots/randomobstacles.h>
#include <ode_robots/terrainground.h>


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
	double friction_first_body;  /** velocity depending friction factor */
	std::string env = "no";  /** "wall", "playground", "halfpipe" or "no" */
	bool randObstacles = true;
 	Pos initPos;


    ThisSim() {
   	  addPaletteFile("colors/UrbanExtraColors.gpl");
   	  addColorAliasFile("colors/UrbanColorSchema.txt");
   	  addColorAliasFile("colors.txt");
   	  setGroundTexture("Images/whiteground.jpg");
    }


    virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
    {
	  /******** GLOBAL SETTINGS **********/

      global.odeConfig.setParam("simstepsize", 0.001);
      global.odeConfig.setParam("controlinterval", 1);
      global.odeConfig.setParam("gravity", -9.8);
	  global.odeConfig.setParam("noise", 0);
	  global.odeConfig.addParameterDef("friction", &friction, 0.1, "parameter for velocity depending friction");
	  global.odeConfig.addParameterDef("friction_first_body", &friction_first_body, 0, " 0 or 1, if 0 friction is applied to all robots of the chain");
	  /******* END GLOBAL SETTINGS *********/



	  /********* ENVIRONMENT **********/
	  if( env == "wall" )
	  { 
	    auto* wall1 = new Box(15,0.3,1.5);
	    wall1->init( odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
	    wall1->setSubstance(  Substance::getPlastic(0.8) );   //  roughness
	    //wall1->setSubstance(  Substance::getRubber(25) ); // hardness 
	    //wall1->setSubstance(  Substance::getFoam(1) );  // hardness (elasticity 0)
	    wall1->setPosition( Pos(0,30,0) );
        auto* wall2 = new Box(15,0.3,1.5);
	    wall2->init( odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
	    wall2->setSubstance(  Substance::getPlastic(0.8) );   //  roughness
	    //wall2->setSubstance(  Substance::getRubber(25) ); // hardness 
	    //wall2->setSubstance(  Substance::getFoam(1) );  // hardness (elasticity 0)
	    wall2->setPosition( Pos(0,-10,0) );

        initPos = Pos(0,0,0);
        setCameraHomePos(Pos(0.00853173, -2.64447, 0.673756),  Pos(-0.251526, -8.93542, 0));
        setCameraMode(Follow);
	  }
      if( env == "boxes" )
      {  
        /** läuft noch nicht */
        //auto* box = new Box(0.2,0.2,0.2);
	    //box->init( odeHandle, 2, osgHandle );
	    //box->setSubstance(  Substance::getPlastic(0.8) );   //  roughness
	    ////box->setSubstance(  Substance::getRubber(25) ); // hardness 
	    ////box->setSubstance(  Substance::getFoam(1) );  // hardness (elasticity 0)
	    //box->setPosition( Pos(0,10,0) );
        //setCameraHomePos(Pos(0.0352671, -2.84712, 0.905692),  Pos(1.41342, -10.5767, 0));
      }
      if( env == "halfpipe" )
      {
        TerrainGround* pipe = new TerrainGround( odeHandle, osgHandle, "terrains/dip128.ppm",
                                                 "terrains/dip128_texture.ppm", 10, 1000, 1,
                                                 OSGHeightField::Red);
        pipe->setPose( osg::Matrix::translate(0,0,0) );
        global.obstacles.push_back( pipe );
        initPos = Pos(0,0,0.1);
        setCameraHomePos (Pos(2.90049, 11.4115, 2.62329),  Pos(169.839, -11.4999, 0));
        setCameraMode(Follow);
      }
      if( env == "slope" )
      {
        double slope = 0.3;
        auto* box = new Box(30,100,3);
        //box->setTexture("Images/whiteground.jpg");
        box->setTexture("Images/wood.jpg");
        box->init( odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
        box->setSubstance(Substance::getPlastic(0.8));
        box->setPose(osg::Matrix::rotate(slope,0.,1.,0.)); //member function of Box class
        auto* box_right = new Box(30,100,3);
        //box_right->setTexture("Images/whiteground.jpg");
        box_right->setTexture("Images/wood.jpg");
        box_right->init( odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
        box_right->setSubstance(Substance::getPlastic(0.8));
        box_right->setPose(osg::Matrix::rotate(slope,0.,-1.,0.)
                          *osg::Matrix::translate(40,0,0)); //member function of Box class
        auto* box_front = new Box(70,30,3);
        //box_front->setTexture("Images/whiteground.jpg");
        box_front->setTexture("Images/wood.jpg");
        box_front->init( odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
        box_front->setSubstance(Substance::getPlastic(0.8));
        box_front->setPose(osg::Matrix::rotate(slope,1.,0.,0.)
                          *osg::Matrix::translate(20,38,0)); //member function of Box class
        auto* box_back = new Box(70,30,3);
        //box_back->setTexture("Images/whiteground.jpg");
        box_back->setTexture("Images/wood.jpg");
        box_back->init( odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
        box_back->setSubstance(Substance::getPlastic(0.8));
        box_back->setPose(osg::Matrix::rotate(slope,-1.,0.,0.)
                          *osg::Matrix::translate(20,-38,0)); //member function of Box class
        initPos = Pos(0,0,2.5);    
        //setCameraHomePos(Pos(1.53111, -6.17297, 2.54418),  Pos(5.12451, -3.32855, 0));
        //setCameraHomePos(Pos(0.776101, -3.97541, 2.27557),  Pos(2.31992, -6.84192, 0));
        setCameraHomePos(Pos(-5.51694, -11.5603, 6.56456),  Pos(-22.7518, -13.0505, 0));
        setCameraMode(Follow);
       }
	  if( env == "playground" )
	  { 
		OdeHandle wallHandle = odeHandle;
		//wallHandle.substance.toPlastic(0.8);
		wallHandle.substance.toRubber(25);
        OctaPlayground* world = new OctaPlayground( wallHandle, osgHandle, 
													Pos(10,0.2,0.5), 8, false);
        world->setPose( osg::Matrix::translate(0,0,0) );
        global.obstacles.push_back( world );
        initPos = Pos(0,0,0);
        //setCameraHomePos(Pos(0.149249, 0.434834, 30.4279),  Pos(179.487, -89.6461, 0));
        //setCameraHomePos(Pos(6.80232, 46.9342, 34.5286),  Pos(173.018, -40.2009, 0));
        setCameraHomePos(Pos(-0.671052, 18.1675, 22.2259),  Pos(-179.845, -53.4133, 0));
        setCameraMode( Static );
	
		RandomObstaclesConf randConf = RandomObstacles::getDefaultConf();
    	randConf.pose = osg::Matrix::translate(0,0,0);
    	//randConf.area = Pos(8,8,2);
    	//randConf.minSize = Pos(0.1,0.1,0.8);
    	//randConf.maxSize = Pos(1.,1.,0.8);
    	//randConf.minDensity = 2;
    	//randConf.maxDensity = 5;
    	randConf.area = Pos(5,5,2);
    	randConf.minSize = Pos(0.1,0.1,0.4);
    	randConf.maxSize = Pos(0.7,0.7,0.4);
    	randConf.minDensity = 2;
    	randConf.maxDensity = 5;
    	RandomObstacles* RandObstacle = new RandomObstacles(wallHandle, osgHandle, randConf);
    	/** Generation an placing Objects */
    	if(randObstacles == true){
          int num_randObs = 16;
          for (int i=0; i< num_randObs; i++){
             RandObstacle->spawn(RandomObstacles::Box, RandomObstacles::Foam);
             global.obstacles.push_back( RandObstacle );
          }
    	}
        setCameraHomePos (Pos(0.501969, 1.09391, 0.340535),  Pos(157.028, -10.8405, 0));
	    setCameraMode( Follow );
	  }
      if( env == "no" )
      {
        setCameraHomePos(Pos(-2.24309, 0.920369, 1.45178),  Pos(-113.025, -29.863, 0));
        setCameraHomePos (Pos(0.501969, 1.09391, 0.340535),  Pos(157.028, -10.8405, 0));
	    setCameraMode( Follow );
      }
	  /********* End ENVIRONMENT *********/
  



      /********* CAR CHAIN ***********/
      CarChainConf conf = CarChain::getDefaultConf();
      conf.carNumber     = 5;
      (conf.carNumber == 1 ) ? conf.supportWheels = true : conf.supportWheels = false;
      conf.randomInitWP  = false;
      auto robot = new CarChain( odeHandle, osgHandle, global.odeConfig, conf, "Train");
   	  //robot->place(Pos(0, 0, 0));
   	  //robot->place(Pos(0, 0, 2));
      robot->place( initPos );
   	  auto controller = new CouplingRod("Coupling_Rod", global.odeConfig);
   	  auto wiring = new One2OneWiring(new WhiteNormalNoise());
   	  auto agent = new OdeAgent(global);
   	  //auto agent = new OdeAgent( PlotOption(File) );
   	  agent->init(controller, robot, wiring);
   	  global.agents.push_back(agent);
   	  global.configs.push_back(agent);
      TrackRobot* TrackOpt = new TrackRobot(false,false,false,true);
      TrackOpt->conf.displayTraceDur = 400;
      TrackOpt->conf.displayTraceThickness = 0.01;
      agent->setTrackOptions( *TrackOpt );
	  /*********** END CAR CHAIN **********/



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


    }


    virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
	  if(!pause)
	  {
		 OdeRobot* rob = globalData.agents[0]->getRobot();
         if( friction_first_body ) {
		   Pos vel = rob->getMainPrimitive()->getVel();
		   rob->getMainPrimitive()->applyForce(-vel*friction);
         }
         else {
            dynamic_cast<CarChain*>(rob)->velocityFriction( friction );
         }
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
		  case 'l': dBodyAddTorque( rob->getMainPrimitive()->getBody(), 0, 0, 5); break;
		  case 'L': dBodyAddTorque( rob->getMainPrimitive()->getBody(), 0, 0, -5); break;
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
